#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPAsyncWebServer.h>
#include <DHT.h>
#include <Preferences.h>
#include <time.h>
#include <ESPmDNS.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <PubSubClient.h>

// =====================================================================
// הגדרות פינים
// =====================================================================
#define D0_PIN           5   // שונה מ-12 — GPIO12 בעיית Boot
#define D1_PIN           13
#define TOUCH_PIN        32
#define BOOST_12V_EN_PIN 14
#define LOCK_SSR         25
#define DHTPIN           4
#define DHTTYPE          DHT11
#define BAT_ADC          35
#define LED_RED_PIN      19
#define LED_GREEN_PIN    18

// =====================================================================
// הגדרות זמנים
// =====================================================================
#define LOCK_OPEN_MS        15000
#define LOCK_COOLDOWN_MS    2000
#define READER_TIMEOUT_MS   15000
#define WIEGAND_TIMEOUT_US  200000
#define BOOSTER_SETTLING_MS 80   // 200ms גרם לקריאת 11 ביטים בלבד כי התגית מגיבה בt≈170ms

// סרוו SG90 — LEDC 50Hz, רזולוציה 16-bit (מחזור 20ms = 65535 טיקים)
// SERVO_PIN מוגדר כ-LOCK_SSR (GPIO25) — אותו פין, פרוטוקול PWM במקום SSR
#define SERVO_CH         0       // LEDC channel
#define SERVO_FREQ_HZ    50
#define SERVO_RES_BITS   16
#define SERVO_CR_STOP    4915   // 1.5ms — עצירה מוחלטת
#define SERVO_CR_UNLOCK  3276   // 1.0ms — סיבוב לכיוון פתיחה
#define SERVO_CR_LOCK    6553   // 2.0ms — סיבוב לכיוון נעילה
// אם הנעל מסתובבת לכיוון ההפוך — פשוט החלף את ערכי UNLOCK ו-LOCK
#define SERVO_MOVE_MS    850    // כוונן: מספיק ms עד שהתפס נועל/נפתח לגמרי
#define SLEEP_TIMEOUT_MS    60000

// דיווח MQTT אחת לשעה (deep-sleep timer)
#define REPORT_INTERVAL_US      (3600ULL * 1000000ULL)
#define WIFI_CONNECT_TIMEOUT_MS 10000

// מדידת סוללה
#define BAT_SAMPLE_MS    10000UL  // דגימה כל 10 שניות
#define BAT_SAMPLES      6        // 6 דגימות = חלון של דקה
#define BAT_OUTLIER_V    0.3f     // סטייה מקסימלית מה-median כדי להיחשב תקין
#define MQTT_PERIODIC_MS (10UL * 60UL * 1000UL)  // MQTT ל-HA כל 10 דקות

// =====================================================================
// הגדרות רשת
// =====================================================================
// רשתות WiFi — WiFiMulti מתחבר לחזקה שזמינה
const char* ssid         = "Kedmi";
const char* password     = "0504241190";
const char* ssid2        = "yosss";
const char* password2    = "0547591866";
const char* www_username = "admin";
const char* www_password = "12345678";
const char* api_token    = "sbs_secure_99";

// MQTT — עדכן את ה-IP של ה-HA שלך
const char* mqtt_server   = "10.0.0.30";  // <-- שנה לIP של ה-HA
const int   mqtt_port     = 1883;
const char* mqtt_user     = "SBS_Monitor";
const char* mqtt_pass     = "SBS123";
const char* mqtt_clientid = "smartsafe_pro";

#define MQTT_STATE_TOPIC  "smartsafe/state"
#define MQTT_EVENT_TOPIC  "smartsafe/events"
#define MQTT_AVAIL_TOPIC  "smartsafe/availability"
#define MQTT_PREFIX       "homeassistant"
#define MQTT_DEVICE_ID    "smartsafe_pro"

// =====================================================================
// RTC Memory Buffer — שורד Deep Sleep (8KB זמין, משתמשים ב-~4KB)
// =====================================================================
#define RTC_BUFFER_SIZE 200

struct __attribute__((packed)) SensorRecord {
    uint32_t timestamp_s;  // מאז boot
    float    temp;
    float    hum;
    uint8_t  battery_pct;
    uint8_t  lock_state;   // 0=IDLE, 2=LOCK_OPEN
    uint8_t  event_type;   // 0=periodic, 1=rfid_open, 2=web_open, 3=denied
    uint8_t  reserved;
    uint32_t card_code;
};
// 20 bytes × 200 = 4000 bytes

RTC_DATA_ATTR SensorRecord rtcBuffer[RTC_BUFFER_SIZE];
RTC_DATA_ATTR uint8_t      rtcHead       = 0;
RTC_DATA_ATTR uint8_t      rtcCount      = 0;
RTC_DATA_ATTR uint32_t     bootCount     = 0;

void rtcAddRecord(float t, float h, uint8_t bat, uint8_t lockSt,
                  uint8_t evType, uint32_t cardCode = 0) {
    SensorRecord rec;
    rec.timestamp_s = millis() / 1000;
    rec.temp        = isnan(t) ? -99.0f : t;
    rec.hum         = isnan(h) ? -1.0f  : h;
    rec.battery_pct = bat;
    rec.lock_state  = lockSt;
    rec.event_type  = evType;
    rec.reserved    = 0;
    rec.card_code   = cardCode;
    rtcBuffer[rtcHead] = rec;
    rtcHead = (rtcHead + 1) % RTC_BUFFER_SIZE;
    if (rtcCount < RTC_BUFFER_SIZE) rtcCount++;
}

int rtcGetIdx(int pos) {
    if (rtcCount < RTC_BUFFER_SIZE) return pos;
    return (rtcHead + pos) % RTC_BUFFER_SIZE;
}

// =====================================================================
// מצבי מערכת
// =====================================================================
enum SystemState  { IDLE, READER_ACTIVE, LOCK_OPEN };
enum WakeReason   { WAKE_BOOT, WAKE_TOUCH, WAKE_TIMER };

SystemState currentState = IDLE;
WakeReason  wakeReason   = WAKE_BOOT;

struct WiegandData {
    volatile uint32_t code;
    volatile int      bits;
    volatile unsigned long lastMicros;
};
WiegandData v_rfid = {0, 0, 0};

unsigned long stateTimer       = 0;
unsigned long lastUnlockTime   = 0;
unsigned long boosterStartTime = 0;
unsigned long activityTimer    = 0;
bool     editMode  = false;
uint32_t masterKey = 10311717;

DHT            dht(DHTPIN, DHTTYPE);
AsyncWebServer server(80);
Preferences    prefs;
WiFiClient     wifiClient;
WiFiMulti      wifiMulti;
PubSubClient   mqttClient(wifiClient);

float         cachedTemp  = NAN;
float         cachedHum   = NAN;
unsigned long lastDHTRead = 0;

float         batSamples[BAT_SAMPLES] = {};
int           batSampleIdx    = 0;
float         batVAvg         = NAN;   // מיצוע מסונן של מתח הסוללה
int           batPctAvg       = -1;    // אחוז מחושב מהמיצוע
unsigned long lastBatSampleMs = 0;
unsigned long lastMqttMs      = 0;

// ערכי סרוו בזמן ריצה — נטענים מ-Preferences, ניתנים לכיול דרך /calib
uint32_t      servoUnlockDuty = SERVO_CR_UNLOCK;
uint32_t      servoLockDuty   = SERVO_CR_LOCK;
int           servoMoveMs     = SERVO_MOVE_MS;
volatile unsigned long calibStopAt = 0;  // לביטול סיבוב כיול בלי delay

// =====================================================================
// Activity Log
// =====================================================================
#define LOG_SIZE 15
struct LogEntry { char msg[52]; unsigned long ts; };
LogEntry actLog[LOG_SIZE];
int logHead = 0, logCount = 0;

void addLog(const char* msg) {
    strlcpy(actLog[logHead].msg, msg, sizeof(actLog[0].msg));
    actLog[logHead].ts = millis();
    logHead = (logHead + 1) % LOG_SIZE;
    if (logCount < LOG_SIZE) logCount++;
}

static void fmtUptime(unsigned long ms, char* buf, size_t len) {
    unsigned long s = ms / 1000;
    snprintf(buf, len, "%02lu:%02lu:%02lu", s / 3600, (s % 3600) / 60, s % 60);
}

// =====================================================================
// סוללה
// =====================================================================
struct BatteryInfo { int raw; float vbat; int pct; };

BatteryInfo getBatteryInfo() {
    BatteryInfo b;
    b.raw  = analogRead(BAT_ADC);
    b.vbat = (b.raw / 4095.0f) * 3.3f * 2.0f;
    b.pct  = constrain((int)((b.vbat - 3.0f) / 1.2f * 100.0f), 0, 100);
    return b;
}

int getBatteryPct() { return getBatteryInfo().pct; }

// מחשב מיצוע של מערך דגימות תוך סינון outliers לפי median
static float filteredBatAverage(float* arr, int n) {
    float s[BAT_SAMPLES];
    memcpy(s, arr, n * sizeof(float));
    for (int i = 1; i < n; i++) {
        float k = s[i]; int j = i - 1;
        while (j >= 0 && s[j] > k) { s[j+1] = s[j]; j--; }
        s[j+1] = k;
    }
    float med = (n % 2 == 0) ? (s[n/2-1] + s[n/2]) / 2.0f : s[n/2];
    float sum = 0.0f; int cnt = 0;
    for (int i = 0; i < n; i++)
        if (fabsf(arr[i] - med) <= BAT_OUTLIER_V) { sum += arr[i]; cnt++; }
    return (cnt > 0) ? sum / cnt : med;
}

// =====================================================================
// כרטיסים
// =====================================================================
void cardAdd(uint32_t code) {
    char key[16]; sprintf(key, "k_%u", code);
    if (!prefs.isKey(key)) prefs.putUInt(key, code);
    String list    = prefs.isKey("cardlist") ? prefs.getString("cardlist", "") : "";
    String codeStr = String(code);
    bool inList = (list == codeStr)
               || list.startsWith(codeStr + ",")
               || list.endsWith("," + codeStr)
               || list.indexOf("," + codeStr + ",") != -1;
    if (!inList) {
        if (list.length() > 0) list += ",";
        list += codeStr;
        prefs.putString("cardlist", list);
        Serial.printf("[CARDS] saved: %s\n", list.c_str());
    }
}

void cardRemove(uint32_t code) {
    char key[16]; sprintf(key, "k_%u", code);
    prefs.remove(key);
    String list    = prefs.isKey("cardlist") ? prefs.getString("cardlist", "") : "";
    String newList = "", codeStr = String(code);
    int start = 0, end;
    while ((end = list.indexOf(',', start)) != -1) {
        String tok = list.substring(start, end);
        if (tok != codeStr) { if (newList.length()) newList += ","; newList += tok; }
        start = end + 1;
    }
    String last = list.substring(start);
    if (last != codeStr) { if (newList.length()) newList += ","; newList += last; }
    prefs.putString("cardlist", newList);
}

bool cardExists(uint32_t code) {
    char key[16]; sprintf(key, "k_%u", code);
    return prefs.isKey(key);
}

// =====================================================================
// MQTT
// =====================================================================
void mqttPublishDiscovery() {
    // טמפרטורה
    mqttClient.publish(
        MQTT_PREFIX "/sensor/" MQTT_DEVICE_ID "/temperature/config",
        "{\"name\":\"SmartSafe Temperature\","
        "\"state_topic\":\"" MQTT_STATE_TOPIC "\","
        "\"value_template\":\"{{ value_json.temp }}\","
        "\"unit_of_measurement\":\"°C\","
        "\"device_class\":\"temperature\","
        "\"unique_id\":\"sbs_temp\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\",\"model\":\"ESP32\"}}",
        true);

    // לחות
    mqttClient.publish(
        MQTT_PREFIX "/sensor/" MQTT_DEVICE_ID "/humidity/config",
        "{\"name\":\"SmartSafe Humidity\","
        "\"state_topic\":\"" MQTT_STATE_TOPIC "\","
        "\"value_template\":\"{{ value_json.hum }}\","
        "\"unit_of_measurement\":\"%\","
        "\"device_class\":\"humidity\","
        "\"unique_id\":\"sbs_hum\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\"}}",
        true);

    // סוללה
    mqttClient.publish(
        MQTT_PREFIX "/sensor/" MQTT_DEVICE_ID "/battery/config",
        "{\"name\":\"SmartSafe Battery\","
        "\"state_topic\":\"" MQTT_STATE_TOPIC "\","
        "\"value_template\":\"{{ value_json.battery }}\","
        "\"unit_of_measurement\":\"%\","
        "\"device_class\":\"battery\","
        "\"unique_id\":\"sbs_battery\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\"}}",
        true);

    // מצב נעילה
    mqttClient.publish(
        MQTT_PREFIX "/binary_sensor/" MQTT_DEVICE_ID "/lock/config",
        "{\"name\":\"SmartSafe Lock\","
        "\"state_topic\":\"" MQTT_STATE_TOPIC "\","
        "\"value_template\":\"{{ value_json.locked }}\","
        "\"payload_on\":\"true\","
        "\"payload_off\":\"false\","
        "\"device_class\":\"lock\","
        "\"unique_id\":\"sbs_lock\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\"}}",
        true);

    // אירועים
    mqttClient.publish(
        MQTT_PREFIX "/sensor/" MQTT_DEVICE_ID "/last_event/config",
        "{\"name\":\"SmartSafe Last Event\","
        "\"state_topic\":\"" MQTT_EVENT_TOPIC "\","
        "\"unique_id\":\"sbs_event\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\"}}",
        true);

    // boot count
    mqttClient.publish(
        MQTT_PREFIX "/sensor/" MQTT_DEVICE_ID "/boot_count/config",
        "{\"name\":\"SmartSafe Boot Count\","
        "\"state_topic\":\"" MQTT_STATE_TOPIC "\","
        "\"value_template\":\"{{ value_json.boot_count }}\","
        "\"entity_category\":\"diagnostic\","
        "\"unique_id\":\"sbs_boot\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\"}}",
        true);

    Serial.println("[MQTT] Discovery published");
}

void mqttPublishState(float temp, float hum, int bat, bool locked) {
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"temp\":%.1f,\"hum\":%.0f,\"battery\":%d,\"locked\":%s,\"boot_count\":%u}",
        isnan(temp) ? -99.0f : temp,
        isnan(hum)  ? -1.0f  : hum,
        bat, locked ? "true" : "false", bootCount);
    mqttClient.publish(MQTT_STATE_TOPIC, payload, true);
    Serial.printf("[MQTT] State: %s\n", payload);
}

void mqttPublishEvent(const char* eventType, uint32_t cardCode = 0) {
    char payload[128];
    if (cardCode > 0)
        snprintf(payload, sizeof(payload),
            "{\"event\":\"%s\",\"card\":%u,\"uptime\":%lu}", eventType, cardCode, millis()/1000);
    else
        snprintf(payload, sizeof(payload),
            "{\"event\":\"%s\",\"uptime\":%lu}", eventType, millis()/1000);
    mqttClient.publish(MQTT_EVENT_TOPIC, payload, false);
    Serial.printf("[MQTT] Event: %s\n", payload);
}

void mqttFlushBuffer() {
    if (rtcCount == 0) return;
    Serial.printf("[MQTT] Flushing %u buffered records\n", rtcCount);
    for (int i = 0; i < rtcCount; i++) {
        int idx = rtcGetIdx(i);
        SensorRecord& rec = rtcBuffer[idx];
        char payload[256];
        snprintf(payload, sizeof(payload),
            "{\"temp\":%.1f,\"hum\":%.0f,\"battery\":%u,\"locked\":%s,"
            "\"boot_count\":%u,\"buffered\":true,\"ts\":%u}",
            rec.temp, rec.hum, rec.battery_pct,
            (rec.lock_state == 0) ? "true" : "false",
            bootCount, rec.timestamp_s);
        mqttClient.publish(MQTT_STATE_TOPIC, payload, true);

        if (rec.event_type == 1) {
            char ev[128];
            snprintf(ev, sizeof(ev),
                "{\"event\":\"rfid_open\",\"card\":%u,\"buffered\":true,\"ts\":%u}",
                rec.card_code, rec.timestamp_s);
            mqttClient.publish(MQTT_EVENT_TOPIC, ev, false);
        } else if (rec.event_type == 2) {
            char ev[128];
            snprintf(ev, sizeof(ev),
                "{\"event\":\"web_open\",\"buffered\":true,\"ts\":%u}", rec.timestamp_s);
            mqttClient.publish(MQTT_EVENT_TOPIC, ev, false);
        } else if (rec.event_type == 3) {
            char ev[128];
            snprintf(ev, sizeof(ev),
                "{\"event\":\"denied\",\"card\":%u,\"buffered\":true,\"ts\":%u}",
                rec.card_code, rec.timestamp_s);
            mqttClient.publish(MQTT_EVENT_TOPIC, ev, false);
        }
        mqttClient.loop();
        delay(10);
    }
    rtcCount = 0;
    rtcHead  = 0;
    Serial.println("[MQTT] Buffer cleared");
}

bool connectMQTT() {
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setBufferSize(512);
    for (int i = 0; i < 3; i++) {
        if (mqttClient.connect(mqtt_clientid, mqtt_user, mqtt_pass)) {
            Serial.println("[MQTT] Connected");
            return true;
        }
        Serial.printf("[MQTT] Failed rc=%d, retry %d/3\n", mqttClient.state(), i+1);
        delay(500);
    }
    return false;
}

// =====================================================================
// HTML Dashboard
// =====================================================================
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="he" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>SmartSafe Pro</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--s2:#1a1a2e;--bd:#2a2a45;--ac:#6366f1;--ag:rgba(99,102,241,.3);--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:480px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:18px}
.hdr h1{font-size:1.25rem;font-weight:700;letter-spacing:-.02em}
.hdr .sub{color:var(--tm);font-size:.7rem;margin-top:2px}
.badge{display:inline-flex;align-items:center;gap:5px;padding:4px 11px;border-radius:20px;font-size:.72rem;font-weight:600;background:rgba(16,185,129,.12);color:var(--gr);border:1px solid rgba(16,185,129,.25)}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:14px;transition:border-color .25s}
.card:hover{border-color:var(--ac)}
.lbl{color:var(--tm);font-size:.65rem;text-transform:uppercase;letter-spacing:.08em;margin-bottom:7px}
.val{font-size:1.55rem;font-weight:700;line-height:1}
.sub2{color:var(--tm);font-size:.72rem;margin-top:4px}
.lock-card{grid-column:span 2;display:flex;align-items:center;gap:18px}
.lock-ico{width:62px;height:62px;border-radius:14px;display:flex;align-items:center;justify-content:center;font-size:1.9rem;flex-shrink:0;transition:all .4s}
.lock-ico.lk{background:rgba(239,68,68,.12);border:1px solid rgba(239,68,68,.28)}
.lock-ico.ul{background:rgba(16,185,129,.12);border:1px solid rgba(16,185,129,.28);animation:pg 1s ease-in-out 3}
@keyframes pg{0%,100%{box-shadow:0 0 0 0 rgba(16,185,129,0)}50%{box-shadow:0 0 0 8px rgba(16,185,129,.2)}}
.lock-h2{font-size:1.05rem;margin-bottom:3px}
.lock-p{color:var(--tm);font-size:.78rem}
.bat-bar{height:7px;background:var(--bd);border-radius:4px;margin-top:7px;overflow:hidden}
.bat-fill{height:100%;border-radius:4px;transition:width .5s}
.bat-fill.hi{background:var(--gr)}.bat-fill.md{background:var(--yw)}.bat-fill.lo{background:var(--rd)}
.wifi{display:flex;align-items:flex-end;gap:3px;height:22px;margin-bottom:4px}
.wifi span{width:5px;background:var(--bd);border-radius:2px;transition:background .3s}
.wifi span:nth-child(1){height:6px}.wifi span:nth-child(2){height:11px}.wifi span:nth-child(3){height:16px}.wifi span:nth-child(4){height:21px}
.wifi.gd span{background:var(--gr)}.wifi.md span:not(:nth-child(4)){background:var(--yw)}.wifi.wk span:first-child{background:var(--rd)}
.btn-ul{width:100%;padding:15px;background:linear-gradient(135deg,#6366f1,#8b5cf6);border:none;border-radius:12px;color:#fff;font-size:.98rem;font-weight:700;cursor:pointer;transition:all .2s;margin-bottom:10px;letter-spacing:.02em}
.btn-ul:hover{transform:translateY(-1px);box-shadow:0 8px 25px var(--ag)}
.btn-ul:active{transform:translateY(0)}
.btn-ul:disabled{opacity:.45;cursor:not-allowed;transform:none}
.sec{font-size:.68rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin:18px 0 7px}
.lst{background:var(--s1);border:1px solid var(--bd);border-radius:12px;overflow:hidden}
.li{display:flex;align-items:center;justify-content:space-between;padding:11px 14px;border-bottom:1px solid var(--bd);font-size:.83rem}
.li:last-child{border-bottom:none}
.li .id{color:var(--tm);font-family:monospace;font-size:.72rem}
.btn-del{background:rgba(239,68,68,.1);border:1px solid rgba(239,68,68,.28);color:var(--rd);padding:4px 10px;border-radius:6px;font-size:.72rem;cursor:pointer;transition:all .2s}
.btn-del:hover{background:rgba(239,68,68,.2)}
.log-it{padding:9px 14px;border-bottom:1px solid var(--bd);font-size:.78rem}
.log-it:last-child{border-bottom:none}
.log-t{color:var(--tm);font-family:monospace;font-size:.68rem;margin-bottom:2px}
.log-it.ok .log-m{color:var(--gr)}.log-it.er .log-m{color:var(--rd)}.log-it.in .log-m{color:var(--tx)}
.spin{display:inline-block;width:13px;height:13px;border:2px solid var(--bd);border-top-color:var(--ac);border-radius:50%;animation:sp .8s linear infinite}
@keyframes sp{to{transform:rotate(360deg)}}
.empty{padding:22px;text-align:center;color:var(--tm);font-size:.82rem}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%) translateY(80px);background:var(--s2);border:1px solid var(--bd);padding:11px 22px;border-radius:8px;font-size:.82rem;transition:transform .3s;z-index:99;white-space:nowrap}
.toast.sh{transform:translateX(-50%) translateY(0)}
.toast.ok{border-color:var(--gr);color:var(--gr)}.toast.er{border-color:var(--rd);color:var(--rd)}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#128274; SmartSafe Pro</h1><div class="sub" id="uptime">מחכה לנתונים...</div></div>
  <div class="badge">&#9679; מחובר</div>
</div>
<div class="g2">
  <div class="card lock-card">
    <div class="lock-ico lk" id="lIco">&#128274;</div>
    <div><div class="lock-h2" id="lStat">נעול</div><div class="lock-p" id="lSub">מצב: המתנה</div></div>
  </div>
</div>
<div class="g2">
  <div class="card"><div class="lbl">טמפרטורה</div><div class="val" id="tVal">--</div><div class="sub2">°C</div></div>
  <div class="card"><div class="lbl">לחות</div><div class="val" id="hVal">--</div><div class="sub2">% RH</div></div>
  <div class="card">
    <div class="lbl">סוללה</div>
    <div class="val" id="bVal">--%</div>
    <div class="bat-bar"><div class="bat-fill hi" id="bBar" style="width:0%"></div></div>
    <div id="bDbg" style="font-family:monospace;font-size:.62rem;color:var(--yw);margin-top:5px;line-height:1.5">raw: --<br>-- V</div>
  </div>
  <div class="card">
    <div class="lbl">WiFi</div>
    <div class="wifi gd" id="wBars"><span></span><span></span><span></span><span></span></div>
    <div class="sub2" id="wRssi">-- dBm</div>
  </div>
</div>
<button class="btn-ul" id="ulBtn" onclick="doUnlock()">&#128275; פתח דלת</button>
<div class="sec">כרטיסים מורשים</div>
<div class="lst" id="cList"><div class="empty"><span class="spin"></span></div></div>
<div class="sec">יומן פעילות</div>
<div class="lst" id="lList"><div class="empty"><span class="spin"></span></div></div>
<div class="toast" id="toast"></div>
<script>
const TK='sbs_secure_99';let cd=false;
function toast(m,t='in'){const el=document.getElementById('toast');el.textContent=m;el.className='toast sh '+t;setTimeout(()=>el.classList.remove('sh'),3200);}
async function fetchSt(){
  try{
    const d=await(await fetch('/api/status')).json();
    const lk=d.state!=='LOCK_OPEN';
    const ic=document.getElementById('lIco');
    ic.className='lock-ico '+(lk?'lk':'ul');ic.textContent=lk?'🔒':'🔓';
    document.getElementById('lStat').textContent=lk?'נעול':'פתוח';
    const stMap={IDLE:'המתנה',READER_ACTIVE:'קורא פעיל',LOCK_OPEN:'דלת פתוחה'};
    document.getElementById('lSub').textContent='מצב: '+(stMap[d.state]||d.state);
    if(d.temp>-40){document.getElementById('tVal').textContent=d.temp.toFixed(1);document.getElementById('hVal').textContent=d.hum.toFixed(0);}
    const b=Math.min(100,Math.max(0,d.battery));
    document.getElementById('bVal').textContent=b+'%';
    const bf=document.getElementById('bBar');bf.style.width=b+'%';
    bf.className='bat-fill '+(b>50?'hi':b>20?'md':'lo');
    if(d.bat_raw!==undefined)document.getElementById('bDbg').innerHTML='raw: '+d.bat_raw+'<br>'+parseFloat(d.bat_v).toFixed(3)+' V';
    document.getElementById('wRssi').textContent=d.rssi+' dBm';
    const wb=document.getElementById('wBars');
    wb.className='wifi '+(d.rssi>-60?'gd':d.rssi>-75?'md':'wk');
    const s=d.uptime,h=Math.floor(s/3600),m=Math.floor((s%3600)/60);
    document.getElementById('uptime').textContent='פעיל: '+(h?h+'ש׳ ':'')+m+'ד׳';
  }catch(e){}
}
async function fetchCards(){
  try{
    const d=await(await fetch('/api/cards')).json();
    const el=document.getElementById('cList');
    if(!d.cards||!d.cards.length){el.innerHTML='<div class="empty">אין כרטיסים רשומים</div>';return;}
    el.innerHTML=d.cards.map(c=>`<div class="li"><div><div>כרטיס RFID</div><div class="id">#${c}</div></div><button class="btn-del" onclick="delCard('${c}')">הסר</button></div>`).join('');
  }catch(e){}
}
async function fetchLog(){
  try{
    const d=await(await fetch('/api/log')).json();
    const el=document.getElementById('lList');
    if(!d.log||!d.log.length){el.innerHTML='<div class="empty">אין אירועים</div>';return;}
    el.innerHTML=d.log.map(e=>{
      const c=e.msg.includes('נפתח')||e.msg.includes('Open')?'ok':e.msg.includes('נדחה')||e.msg.includes('Denied')||e.msg.includes('Invalid')?'er':'in';
      return`<div class="log-it ${c}"><div class="log-t">${e.time}</div><div class="log-m">${e.msg}</div></div>`;
    }).join('');
  }catch(e){}
}
async function doUnlock(){
  if(cd)return;const btn=document.getElementById('ulBtn');btn.disabled=true;cd=true;
  try{
    const r=await fetch('/open?t='+TK);
    if(r.ok){toast('הדלת נפתחה!','ok');setTimeout(()=>{fetchSt();fetchLog();},600);}
    else if(r.status===429)toast('ממתין לcooldown...','er');
    else toast('שגיאה בפתיחה','er');
  }catch(e){toast('שגיאת חיבור','er');}
  setTimeout(()=>{btn.disabled=false;cd=false;},3000);
}
async function delCard(id){
  if(!confirm('למחוק כרטיס #'+id+'?'))return;
  try{
    const r=await fetch('/api/cards/delete?id='+id,{method:'POST'});
    if(r.ok){toast('כרטיס הוסר','ok');fetchCards();}else toast('שגיאה','er');
  }catch(e){toast('שגיאת חיבור','er');}
}
fetchSt();fetchCards();fetchLog();
setInterval(fetchSt,5000);setInterval(fetchLog,10000);setInterval(fetchCards,8000);
</script>
</body>
</html>
)rawliteral";

const char MONITOR_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="he" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Monitor — SmartSafe</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--bd:#2a2a45;--ac:#6366f1;--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--cy:#06b6d4;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:520px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:16px}
.hdr h1{font-size:1.15rem;font-weight:700}.hdr .sub{color:var(--tm);font-size:.68rem;margin-top:2px}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.sec{font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin:14px 0 6px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:14px;margin-bottom:8px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px}
.lbl{color:var(--tm);font-size:.6rem;text-transform:uppercase;letter-spacing:.08em;margin-bottom:4px}
.val{font-size:1.35rem;font-weight:700;font-family:monospace}
.row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid rgba(42,42,69,.6);font-size:.8rem}
.row:last-child{border-bottom:none}
.mono{font-family:monospace}
.badge{display:inline-flex;align-items:center;gap:6px;padding:5px 13px;border-radius:20px;font-size:.72rem;font-weight:600}
.badge.idle{background:rgba(100,116,139,.15);color:var(--tm);border:1px solid rgba(100,116,139,.3)}
.badge.active{background:rgba(245,158,11,.12);color:var(--yw);border:1px solid rgba(245,158,11,.3)}
.badge.open{background:rgba(16,185,129,.12);color:var(--gr);border:1px solid rgba(16,185,129,.3)}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.35}}
.badge.active{animation:blink .9s ease-in-out infinite}
.dot{width:7px;height:7px;border-radius:50%;display:inline-block}
.dot.on{background:var(--gr)}.dot.off{background:var(--bd)}
.code-big{font-family:monospace;font-size:1.7rem;font-weight:700;color:var(--yw);letter-spacing:.08em}
.bits-sub{color:var(--tm);font-size:.7rem;margin-top:3px}
.slot{display:grid;grid-template-columns:20px 1fr 70px 40px;gap:6px;align-items:center;padding:5px 0;border-bottom:1px solid rgba(42,42,69,.5);font-size:.77rem}
.slot:last-child{border-bottom:none}
.slot-n{color:var(--tm);font-size:.63rem;text-align:center}
.slot-v{font-family:monospace;color:var(--cy)}
.slot-v.empty{color:rgba(42,42,69,.9)}
.slot-pct{color:var(--tm);font-size:.7rem;text-align:left}
.slot-tag{font-size:.62rem;color:var(--tm);text-align:left}
.slot-tag.next{color:var(--yw)}
.slot-tag.ok{color:var(--gr)}
.avg-row{display:flex;align-items:baseline;gap:10px;margin-top:8px;padding-top:8px;border-top:1px solid var(--bd)}
.avg-v{font-size:1.3rem;font-weight:700;font-family:monospace;color:var(--cy)}
.avg-pct{font-size:.9rem;color:var(--tm)}
.card-item{display:flex;justify-content:space-between;padding:7px 0;border-bottom:1px solid rgba(42,42,69,.5);font-size:.8rem}
.card-item:last-child{border-bottom:none}
.card-id{font-family:monospace;color:var(--cy);font-size:.73rem}
.empty{color:var(--tm);font-size:.78rem;text-align:center;padding:12px}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#128270; SmartSafe Monitor</h1><div class="sub" id="upEl">--</div></div>
  <a href="/" class="back">&#8592; דשבורד</a>
</div>

<div class="sec">מצב מכונה</div>
<div class="card" style="display:flex;align-items:center;gap:14px">
  <span id="stBadge" class="badge idle">IDLE</span>
  <span id="stDesc" style="color:var(--tm);font-size:.82rem">המתנה</span>
</div>

<div class="sec">סריקת RFID</div>
<div class="card">
  <div class="lbl" style="display:flex;align-items:center;gap:6px">מזהה שנסרק <span class="dot off" id="scanDot"></span></div>
  <div class="code-big" id="rfidCode">---</div>
  <div class="bits-sub" id="rfidBits">לא פעיל</div>
</div>

<div class="sec">חיישן DHT — נתון גולמי (פלט דיגיטלי ישיר)</div>
<div class="g2">
  <div class="card"><div class="lbl">טמפרטורה</div><div class="val" id="tRaw">--</div><div style="color:var(--tm);font-size:.68rem;margin-top:3px">°C</div></div>
  <div class="card"><div class="lbl">לחות</div><div class="val" id="hRaw">--</div><div style="color:var(--tm);font-size:.68rem;margin-top:3px">% RH</div></div>
</div>
<div class="card">
  <div class="row"><span style="color:var(--tm)">גיל קריאה אחרונה</span><span class="mono" id="dhtAge">--</span></div>
</div>

<div class="sec">מדידת סוללה — נתונים גולמיים ומחושבים</div>
<div class="card">
  <div class="lbl">קריאה חיה מה-ADC</div>
  <div class="row"><span>ADC Raw (0–4095)</span><span class="mono" style="color:var(--yw)" id="batRaw">--</span></div>
  <div class="row"><span>מתח מחושב<span style="color:var(--tm);font-size:.68rem"> (raw/4095×3.3×2)</span></span><span class="mono" style="color:var(--cy)" id="batV">--</span></div>
  <div class="row"><span>אחוז מחושב<span style="color:var(--tm);font-size:.68rem"> ((V-3.0)/1.2×100)</span></span><span class="mono" id="batPct">--</span></div>
</div>
<div class="card">
  <div class="lbl">חלון דגימות — 6 × 10 שניות</div>
  <div id="slotsEl"></div>
  <div class="avg-row">
    <span class="avg-v" id="avgV">--</span>
    <span class="avg-pct" id="avgPct">ממתין לדגימות...</span>
  </div>
</div>

<div class="sec">מפתחות שמורים</div>
<div class="card" id="cardsEl"><div class="empty">טוען...</div></div>

<script>
const stMap={
  IDLE:{label:'IDLE',cls:'idle',desc:'המתנה'},
  READER_ACTIVE:{label:'READER ACTIVE',cls:'active',desc:'קורא RFID פעיל'},
  LOCK_OPEN:{label:'LOCK OPEN',cls:'open',desc:'דלת פתוחה'}
};
function fmtUp(s){const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;return(h?h+'ש׳ ':'')+m+'ד׳ '+sec+'ש"';}
async function refresh(){
  try{
    const d=await(await fetch('/api/monitor')).json();
    document.getElementById('upEl').textContent='פעיל: '+fmtUp(d.uptime);
    const sm=stMap[d.state]||{label:d.state,cls:'idle',desc:''};
    const b=document.getElementById('stBadge');b.textContent=sm.label;b.className='badge '+sm.cls;
    document.getElementById('stDesc').textContent=sm.desc;
    const sc=d.rfid.scanning;
    document.getElementById('scanDot').className='dot '+(sc?'on':'off');
    document.getElementById('rfidCode').textContent=(sc&&d.rfid.bits>0)?String(d.rfid.code):'---';
    document.getElementById('rfidBits').textContent=(sc&&d.rfid.bits>0)?(d.rfid.bits+' bits'):'לא פעיל';
    document.getElementById('tRaw').textContent=d.dht.temp!==null?d.dht.temp.toFixed(1):'ERR';
    document.getElementById('hRaw').textContent=d.dht.hum!==null?d.dht.hum.toFixed(1):'ERR';
    document.getElementById('dhtAge').textContent=(d.dht.age_ms/1000).toFixed(1)+'s';
    document.getElementById('batRaw').textContent=d.bat_live.raw;
    document.getElementById('batV').textContent=d.bat_live.v.toFixed(3)+' V';
    document.getElementById('batPct').textContent=d.bat_live.pct+'%';
    const sl=d.bat_samples,idx=d.bat_sample_idx,ready=d.bat_avg_ready;
    let html='';
    for(let i=0;i<6;i++){
      const v=sl[i],has=v>0.5,isNext=i===idx;
      const pct=has?Math.max(0,Math.min(100,Math.round((v-3.0)/1.2*100))):null;
      html+=`<div class="slot">
        <span class="slot-n">${i+1}</span>
        <span class="slot-v${has?'':' empty'}">${has?v.toFixed(3)+' V':'---'}</span>
        <span class="slot-pct">${pct!==null?pct+'%':''}</span>
        <span class="slot-tag${isNext?' next':has?' ok':''}">${isNext?'← הבא':has?'✓':''}</span>
      </div>`;
    }
    document.getElementById('slotsEl').innerHTML=html;
    if(ready&&d.bat_avg_v!=null){
      document.getElementById('avgV').textContent=d.bat_avg_v.toFixed(3)+' V';
      document.getElementById('avgPct').textContent='ממוצע מסונן · '+d.bat_avg_pct+'%';
    } else {
      document.getElementById('avgV').textContent='--';
      document.getElementById('avgPct').textContent='ממתין לדגימות ('+(ready?6:idx)+'/6)...';
    }
    const cards=d.cards,cel=document.getElementById('cardsEl');
    if(!cards||!cards.length)cel.innerHTML='<div class="empty">אין מפתחות שמורים</div>';
    else cel.innerHTML=cards.map((c,i)=>`<div class="card-item"><span>מפתח ${i+1}</span><span class="card-id">#${c}</span></div>`).join('');
  }catch(e){console.error(e);}
}
refresh();
setInterval(refresh,1000);
</script>
</body>
</html>
)rawliteral";

const char CALIB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="he" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>כיול סרוו</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--bd:#2a2a45;--ac:#6366f1;--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:440px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:16px}
.hdr h1{font-size:1.15rem;font-weight:700}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.sec{font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin:16px 0 6px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:16px;margin-bottom:8px}
.lbl{color:var(--tm);font-size:.65rem;margin-bottom:6px}
.ms-big{font-size:1.6rem;font-weight:700;font-family:monospace;color:var(--yw);margin:4px 0 10px}
input[type=range]{width:100%;accent-color:var(--ac);cursor:pointer;margin-bottom:4px}
input[type=number]{width:100%;padding:10px 12px;background:var(--bg);border:1px solid var(--bd);border-radius:8px;color:var(--tx);font-size:1.1rem;font-family:monospace;margin-bottom:12px;text-align:center}
.row{display:flex;gap:8px;margin-top:10px}
.btn{flex:1;padding:14px 6px;border:none;border-radius:10px;font-size:.9rem;font-weight:700;cursor:pointer;transition:opacity .15s}
.btn:active{opacity:.65}.btn:disabled{opacity:.3;cursor:not-allowed}
.ba{background:rgba(99,102,241,.18);color:#a5b4fc;border:1px solid rgba(99,102,241,.4)}
.bb{background:rgba(245,158,11,.13);color:var(--yw);border:1px solid rgba(245,158,11,.35)}
.bstop{background:rgba(239,68,68,.15);color:var(--rd);border:1px solid rgba(239,68,68,.3);flex:0 0 52px}
.bopen{background:rgba(16,185,129,.12);color:var(--gr);border:1px solid rgba(16,185,129,.3)}
.bsave{width:100%;padding:15px;background:linear-gradient(135deg,#6366f1,#8b5cf6);border:none;border-radius:12px;color:#fff;font-size:1rem;font-weight:700;cursor:pointer;margin-top:4px}
.bsave:disabled{opacity:.3;cursor:not-allowed}
.chip{display:inline-flex;align-items:center;padding:4px 12px;border-radius:20px;font-size:.75rem;font-weight:600;margin-top:10px}
.chip-none{background:rgba(100,116,139,.12);color:var(--tm);border:1px solid rgba(100,116,139,.3)}
.chip-ok{background:rgba(16,185,129,.12);color:var(--gr);border:1px solid rgba(16,185,129,.3)}
.msg{margin-top:10px;padding:10px 12px;border-radius:8px;font-size:.82rem;text-align:center}
.msg-ok{background:rgba(16,185,129,.1);color:var(--gr);border:1px solid rgba(16,185,129,.2)}
.msg-er{background:rgba(239,68,68,.1);color:var(--rd);border:1px solid rgba(239,68,68,.2)}
.step-num{font-size:.65rem;color:var(--ac);font-weight:700;text-transform:uppercase;letter-spacing:.08em}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#9881; כיול סרוו</h1></div>
  <a href="/" class="back">&#8592; דשבורד</a>
</div>

<div class="step-num">שלב 1</div>
<div class="sec">בדוק כיוונים</div>
<div class="card">
  <div class="lbl">משך בדיקה</div>
  <div class="ms-big" id="msDsp">600ms</div>
  <input type="range" id="msSlider" min="100" max="2000" step="50" value="600"
         oninput="document.getElementById('msDsp').textContent=this.value+'ms'">
  <div class="row">
    <button class="btn ba" onclick="spin(3276)">&#8635; כיוון A</button>
    <button class="btn bb" onclick="spin(6553)">&#8634; כיוון B</button>
    <button class="btn bstop" onclick="doStop()" title="עצור">&#9632;</button>
  </div>
</div>

<div class="step-num">שלב 2</div>
<div class="sec">הגדר מה פותח</div>
<div class="card">
  <div class="lbl">לאחר שזיהית — בחר איזה כיוון פותח את הנעל</div>
  <div class="row">
    <button class="btn ba" onclick="setDir('A')">A = פתיחה &#10003;</button>
    <button class="btn bb" onclick="setDir('B')">B = פתיחה &#10003;</button>
  </div>
  <div id="dirChip"><span class="chip chip-none">לא הוגדר עדיין</span></div>
</div>

<div class="step-num">שלב 3</div>
<div class="sec">כוונן משך הסיבוב</div>
<div class="card">
  <div class="lbl">ms עד שהתפס מגיע לסוף — הגדל אם לא מספיק, הקטן אם המנוע נשאר בלחץ</div>
  <input type="number" id="saveMs" min="100" max="3000" step="50" value="600">
  <div class="row">
    <button class="btn bopen" id="btnOpen" onclick="testAction('open')" disabled>&#9654; בדוק פתיחה</button>
    <button class="btn bstop" id="btnClose" onclick="testAction('close')" disabled style="flex:1">&#9654; בדוק נעילה</button>
  </div>
</div>

<div class="step-num">שלב 4</div>
<div class="sec">שמור לפלאש</div>
<div class="card">
  <button class="bsave" id="btnSave" onclick="save()" disabled>&#128190; שמור כיול</button>
  <div id="saveMsg"></div>
</div>

<script>
let unlockDir=null;
function ms(){return parseInt(document.getElementById('msSlider').value);}
function sms(){return parseInt(document.getElementById('saveMs').value);}
async function api(p){try{const r=await fetch('/api/calib?'+p);return await r.json();}catch(e){return{ok:false};}}
function spin(d){api('cmd=spin&duty='+d+'&ms='+ms());}
function doStop(){api('cmd=stop');}
function setDir(d){
  unlockDir=d;
  document.getElementById('dirChip').innerHTML='<span class="chip chip-ok">כיוון '+d+' = פתיחה &#10003;</span>';
  document.getElementById('btnOpen').disabled=false;
  document.getElementById('btnClose').disabled=false;
  document.getElementById('btnSave').disabled=false;
}
function testAction(a){
  const d=(a==='open')?(unlockDir==='A'?3276:6553):(unlockDir==='A'?6553:3276);
  api('cmd=spin&duty='+d+'&ms='+sms());
}
async function save(){
  const r=await api('cmd=save&dir='+unlockDir+'&ms='+sms());
  const el=document.getElementById('saveMsg');
  el.innerHTML=r&&r.ok
    ?'<div class="msg msg-ok">&#10003; נשמר! כיוון '+unlockDir+' פותח &middot; '+sms()+'ms</div>'
    :'<div class="msg msg-er">&#10007; שגיאה בשמירה</div>';
}
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// ISR
// =====================================================================
void IRAM_ATTR rfid_isr() {
    if (currentState != READER_ACTIVE) return;
    int bit = (digitalRead(D1_PIN) == LOW) ? 1 : 0;
    v_rfid.code = (v_rfid.code << 1) | bit;
    v_rfid.bits++;
    v_rfid.lastMicros = micros();
}

// =====================================================================
// FSM
// =====================================================================
void setSystemState(SystemState newState) {
    currentState = newState;
    stateTimer   = millis();
    activityTimer = millis();
    if (newState == IDLE) {
        digitalWrite(BOOST_12V_EN_PIN, LOW);
        static bool firstBoot = true;
        if (firstBoot) {
            firstBoot = false;
            Serial.println("[INIT] Startup calibration: opening...");
            ledcWrite(SERVO_CH, servoUnlockDuty);
            delay(servoMoveMs);
            ledcWrite(SERVO_CH, 0);
            Serial.println("[INIT] Waiting 10 seconds...");
            delay(10000);
            Serial.println("[INIT] Calibration done — locking...");
        }
        ledcWrite(SERVO_CH, servoLockDuty);
        delay(servoMoveMs);
        ledcWrite(SERVO_CH, 0);
        editMode = false;
        Serial.println("[FSM] IDLE");
    } else if (newState == READER_ACTIVE) {
        digitalWrite(BOOST_12V_EN_PIN, HIGH);
        boosterStartTime = millis();
        noInterrupts(); v_rfid.bits = 0; v_rfid.code = 0; interrupts();
        addLog("קורא RFID הופעל");
        Serial.println("[FSM] READER_ACTIVE");
    } else if (newState == LOCK_OPEN) {
        ledcWrite(SERVO_CH, servoUnlockDuty);
        delay(servoMoveMs);
        ledcWrite(SERVO_CH, 0);
        digitalWrite(BOOST_12V_EN_PIN, LOW);
        lastUnlockTime = millis();
        Serial.println("[FSM] LOCK_OPEN");
    }
}

// =====================================================================
// Server Routes
// =====================================================================
void setupServerRoutes() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        req->send_P(200, "text/html", HTML_PAGE);
    });

    server.on("/open", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        if (req->hasParam("t") && req->getParam("t")->value() == api_token) {
            if (millis() - lastUnlockTime > LOCK_COOLDOWN_MS) {
                setSystemState(LOCK_OPEN);
                activityTimer = millis();
                addLog("נפתח מרחוק (ווב)");
                rtcAddRecord(cachedTemp, cachedHum, getBatteryPct(), 2, 2);
                if (mqttClient.connected()) {
                    mqttPublishEvent("web_open");
                    mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), false);
                }
                req->send(200, "text/plain", "OK");
            } else {
                req->send(429, "text/plain", "Cooldown");
            }
        } else {
            addLog("ניסיון פתיחה עם טוקן שגוי");
            req->send(403, "text/plain", "Invalid Token");
        }
    });

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        BatteryInfo bat = getBatteryInfo();
        // עדיפות לערך ממוצע מסונן; fallback לקריאה חיה לפני שיש מספיק דגימות
        int   dispPct  = (batPctAvg >= 0)    ? batPctAvg : bat.pct;
        float dispVolt = !isnan(batVAvg)     ? batVAvg   : bat.vbat;
        int   rssi = WiFi.RSSI();
        const char* st = currentState == LOCK_OPEN     ? "LOCK_OPEN"
                       : currentState == READER_ACTIVE ? "READER_ACTIVE" : "IDLE";
        String json = "{\"state\":\""; json += st;
        json += "\",\"temp\":";    json += isnan(cachedTemp) ? "-99" : String(cachedTemp, 1);
        json += ",\"hum\":";       json += isnan(cachedHum)  ? "-1"  : String(cachedHum,  1);
        json += ",\"battery\":";   json += dispPct;
        json += ",\"bat_raw\":";   json += bat.raw;
        json += ",\"bat_v\":";     json += String(dispVolt, 3);
        json += ",\"rssi\":";      json += rssi;
        json += ",\"uptime\":";    json += millis() / 1000;
        json += ",\"buffered\":";  json += rtcCount;
        json += "}";
        req->send(200, "application/json", json);
    });

    server.on("/api/cards", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        String list = prefs.isKey("cardlist") ? prefs.getString("cardlist", "") : "";
        String json = "{\"cards\":[";
        if (list.length() > 0) {
            int start = 0, end; bool first = true;
            while ((end = list.indexOf(',', start)) != -1) {
                if (!first) json += ",";
                json += "\"" + list.substring(start, end) + "\"";
                first = false; start = end + 1;
            }
            String last = list.substring(start);
            if (last.length() > 0) { if (!first) json += ","; json += "\"" + last + "\""; }
        }
        json += "]}";
        req->send(200, "application/json", json);
    });

    server.on("/api/cards/delete", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        if (!req->hasParam("id")) { req->send(400, "text/plain", "Missing id"); return; }
        uint32_t code = (uint32_t)req->getParam("id")->value().toInt();
        if (!cardExists(code)) { req->send(404, "text/plain", "Not found"); return; }
        cardRemove(code);
        char buf[48]; snprintf(buf, sizeof(buf), "כרטיס #%u הוסר (ווב)", code);
        addLog(buf);
        req->send(200, "text/plain", "OK");
    });

    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        String json = "{\"log\":[";
        int cnt = min(logCount, LOG_SIZE); bool first = true;
        for (int i = cnt - 1; i >= 0; i--) {
            int idx = ((logHead - 1 - i) % LOG_SIZE + LOG_SIZE) % LOG_SIZE;
            char tb[12]; fmtUptime(actLog[idx].ts, tb, sizeof(tb));
            if (!first) json += ",";
            json += "{\"time\":\""; json += tb;
            json += "\",\"msg\":\""; json += actLog[idx].msg; json += "\"}";
            first = false;
        }
        json += "]}";
        req->send(200, "application/json", json);
    });

    server.on("/monitor", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        req->send_P(200, "text/html", MONITOR_PAGE);
    });

    server.on("/calib", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        req->send_P(200, "text/html", CALIB_PAGE);
    });

    server.on("/api/calib", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        String cmd = req->hasParam("cmd") ? req->getParam("cmd")->value() : "";

        if (cmd == "spin") {
            uint32_t duty = req->hasParam("duty") ? (uint32_t)req->getParam("duty")->value().toInt() : 0;
            int ms = req->hasParam("ms") ? constrain(req->getParam("ms")->value().toInt(), 50, 3000) : 600;
            ledcWrite(SERVO_CH, duty);
            calibStopAt = millis() + ms;
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "stop") {
            calibStopAt = 0;
            ledcWrite(SERVO_CH, 0);
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "save") {
            String dir = req->hasParam("dir") ? req->getParam("dir")->value() : "A";
            int ms = req->hasParam("ms") ? constrain(req->getParam("ms")->value().toInt(), 50, 3000) : SERVO_MOVE_MS;
            servoMoveMs     = ms;
            bool swapped    = (dir == "B");
            servoUnlockDuty = swapped ? SERVO_CR_LOCK   : SERVO_CR_UNLOCK;
            servoLockDuty   = swapped ? SERVO_CR_UNLOCK : SERVO_CR_LOCK;
            prefs.putInt ("s_ms",  ms);
            prefs.putBool("s_dir", swapped);
            Serial.printf("[CALIB] saved: dir=%s ms=%d\n", dir.c_str(), ms);
            req->send(200, "application/json", "{\"ok\":true}");

        } else {
            req->send(400, "application/json", "{\"ok\":false}");
        }
    });

    server.on("/api/monitor", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        BatteryInfo bat = getBatteryInfo();
        WiegandData rfid;
        noInterrupts(); rfid = v_rfid; interrupts();

        const char* st = currentState == LOCK_OPEN     ? "LOCK_OPEN"
                       : currentState == READER_ACTIVE ? "READER_ACTIVE" : "IDLE";
        String json = "{\"state\":\""; json += st; json += "\"";

        json += ",\"uptime\":"; json += millis() / 1000;

        // DHT
        json += ",\"dht\":{\"temp\":";
        json += isnan(cachedTemp) ? "null" : String(cachedTemp, 1);
        json += ",\"hum\":";
        json += isnan(cachedHum) ? "null" : String(cachedHum, 1);
        json += ",\"age_ms\":"; json += (millis() - lastDHTRead);
        json += "}";

        // Battery — live reading
        json += ",\"bat_live\":{\"raw\":"; json += bat.raw;
        json += ",\"v\":";   json += String(bat.vbat, 3);
        json += ",\"pct\":"; json += bat.pct;
        json += "}";

        // Battery — sample window
        json += ",\"bat_samples\":[";
        for (int i = 0; i < BAT_SAMPLES; i++) {
            if (i > 0) json += ",";
            json += String(batSamples[i], 3);
        }
        json += "]";
        json += ",\"bat_sample_idx\":"; json += batSampleIdx;
        json += ",\"bat_avg_ready\":";  json += (batPctAvg >= 0 ? "true" : "false");
        if (!isnan(batVAvg)) {
            json += ",\"bat_avg_v\":";   json += String(batVAvg, 3);
            json += ",\"bat_avg_pct\":"; json += batPctAvg;
        }

        // Cards
        json += ",\"cards\":[";
        String list = prefs.isKey("cardlist") ? prefs.getString("cardlist", "") : "";
        if (list.length() > 0) {
            int start = 0, end; bool first = true;
            while ((end = list.indexOf(',', start)) != -1) {
                if (!first) json += ",";
                json += "\"" + list.substring(start, end) + "\"";
                first = false; start = end + 1;
            }
            String last = list.substring(start);
            if (last.length() > 0) { if (!first) json += ","; json += "\"" + last + "\""; }
        }
        json += "]";

        // RFID
        json += ",\"rfid\":{\"scanning\":";
        json += (currentState == READER_ACTIVE ? "true" : "false");
        json += ",\"bits\":"; json += rfid.bits;
        json += ",\"code\":"; json += rfid.code;
        json += "}";

        json += "}";
        req->send(200, "application/json", json);
    });
}

// =====================================================================
// Setup
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    bootCount++;
    Serial.printf("[SYS] Boot #%u\n", bootCount);

    // זיהוי סיבת wake
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if      (cause == ESP_SLEEP_WAKEUP_EXT0)  wakeReason = WAKE_TOUCH;
    else if (cause == ESP_SLEEP_WAKEUP_TIMER) wakeReason = WAKE_TIMER;
    else                                       wakeReason = WAKE_BOOT;

    Serial.printf("[SYS] Wake: %s\n",
        wakeReason == WAKE_TOUCH ? "TOUCH" :
        wakeReason == WAKE_TIMER ? "TIMER" : "BOOT");

    // ── Timer Wake: קרא חיישנים, דווח, חזור לשינה מיד ──────────────
    if (wakeReason == WAKE_TIMER) {
        dht.begin();
        delay(300);
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        int   bat = getBatteryPct();
        Serial.printf("[TIMER] T=%.1f H=%.0f BAT=%d\n", t, h, bat);

        rtcAddRecord(t, h, (uint8_t)bat, 0, 0);

        // נסה WiFi + MQTT
        WiFi.mode(WIFI_STA);
        wifiMulti.addAP(ssid,  password);
        wifiMulti.addAP(ssid2, password2);
        unsigned long wStart = millis();
        while (wifiMulti.run() != WL_CONNECTED && millis() - wStart < WIFI_CONNECT_TIMEOUT_MS)
            delay(200);

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
            if (connectMQTT()) {
                mqttPublishDiscovery();
                mqttFlushBuffer();
                mqttPublishState(t, h, bat, true);
                for (int i = 0; i < 20; i++) { mqttClient.loop(); delay(100); }
                mqttClient.loop();
                delay(200);
                mqttClient.disconnect();
            }
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
        } else {
            Serial.println("[WiFi] Timeout — data buffered for next wake");
        }

        // שינה מושבתת — ממשיך לאתחול רגיל במקום לחזור לשינה
        Serial.println("[SYS] Sleep disabled — continuing to normal boot");
    }

    // ── Touch Wake / Boot: פעולה רגילה ──────────────────────────────
    pinMode(TOUCH_PIN, INPUT_PULLDOWN);
    pinMode(BOOST_12V_EN_PIN, OUTPUT);
    pinMode(D0_PIN, INPUT_PULLUP);
    pinMode(D1_PIN, INPUT_PULLUP);
    digitalWrite(BOOST_12V_EN_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(D0_PIN), rfid_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(D1_PIN), rfid_isr, FALLING);

    prefs.begin("safe-app", false);

    // טעינת כיול סרוו שנשמר
    servoMoveMs     = prefs.getInt("s_ms", SERVO_MOVE_MS);
    bool dirSwapped = prefs.getBool("s_dir", false);
    servoUnlockDuty = dirSwapped ? SERVO_CR_LOCK   : SERVO_CR_UNLOCK;
    servoLockDuty   = dirSwapped ? SERVO_CR_UNLOCK : SERVO_CR_LOCK;

    dht.begin();

    // אתחול סרוו SG90 — GPIO25, 50Hz PWM
    ledcSetup(SERVO_CH, SERVO_FREQ_HZ, SERVO_RES_BITS);
    ledcAttachPin(LOCK_SSR, SERVO_CH);
    ledcWrite(SERVO_CH, 0);  // מפסיק אות — הנעל מחזיקה עצמה, המנוע ישקוט  // עצירה בלבד באתחול — setSystemState(IDLE) ינעל אחר כך

    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(ssid,  password);
    wifiMulti.addAP(ssid2, password2);
    for (int i = 0; i < 20 && wifiMulti.run() != WL_CONNECTED; i++) delay(500);

    if (WiFi.status() == WL_CONNECTED) {
        MDNS.begin("smartsafe");
        Serial.println("=============================");
        Serial.print  ("[WiFi] Connected! IP: ");
        Serial.println(WiFi.localIP());
        Serial.println("[WiFi] URL: http://smartsafe.local");
        Serial.println("=============================");
        if (connectMQTT()) {
            mqttPublishDiscovery();
            mqttFlushBuffer();
        }
    } else {
        Serial.println("[WiFi] Connection FAILED");
    }

    setupServerRoutes();
    server.begin();

    setSystemState(IDLE);
    activityTimer = millis();

    if (wakeReason == WAKE_TOUCH) {
        addLog("התעוררות ממגע");
        Serial.println("[SYS] Wake from Deep Sleep (GPIO32)");
        setSystemState(READER_ACTIVE);
    } else {
        addLog("מערכת הופעלה");
        Serial.println("[SYS] Boot Complete");
    }

    // שינה מושבתת
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
    // עצירת סרוו כיול (non-blocking — במקום delay בתוך ה-HTTP handler)
    if (calibStopAt > 0 && millis() >= calibStopAt) {
        ledcWrite(SERVO_CH, 0);
        calibStopAt = 0;
    }

    // MQTT keepalive
    if (mqttClient.connected()) mqttClient.loop();

    // DHT כל 3 שניות
    if (millis() - lastDHTRead >= 3000) {
        lastDHTRead = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t) && !isnan(h)) {
            cachedTemp = t;
            cachedHum  = h;
            Serial.printf("[DHT] Temp: %.1f°C  Hum: %.0f%%\n", t, h);
        } else {
            Serial.println("[DHT] Read FAILED — check wiring (use 3.3V not 5V!)");
        }
    }

    // דגימת סוללה כל 10 שניות; מיצוע עם סינון outliers אחרי 6 דגימות (דקה)
    // מושעה בזמן READER_ACTIVE — analogRead מייצר רעש ADC שעלול להפריע ל-Wiegand ISR
    if (currentState != READER_ACTIVE && millis() - lastBatSampleMs >= BAT_SAMPLE_MS) {
        lastBatSampleMs = millis();
        batSamples[batSampleIdx++] = getBatteryInfo().vbat;
        if (batSampleIdx >= BAT_SAMPLES) {
            batSampleIdx = 0;
            batVAvg   = filteredBatAverage(batSamples, BAT_SAMPLES);
            batPctAvg = constrain((int)((batVAvg - 3.0f) / 1.2f * 100.0f), 0, 100);
            Serial.printf("[BAT] Avg: %.3fV  %d%%\n", batVAvg, batPctAvg);
        }
    }

    // MQTT ל-HA כל 10 דקות (רק אם יש מיצוע תקין)
    if (batPctAvg >= 0 && mqttClient.connected() &&
        millis() - lastMqttMs >= MQTT_PERIODIC_MS) {
        lastMqttMs = millis();
        mqttPublishState(cachedTemp, cachedHum, batPctAvg, currentState != LOCK_OPEN);
    }

    // Touch
    static bool lastTouch = LOW;
    bool touch = digitalRead(TOUCH_PIN);
    if (touch == HIGH && lastTouch == LOW) {
        activityTimer = millis();
        if (currentState == IDLE)          setSystemState(READER_ACTIVE);
        else if (currentState == READER_ACTIVE) stateTimer = millis();
    }
    lastTouch = touch;

    // RFID
    if (currentState == READER_ACTIVE) {
        if (millis() - boosterStartTime < BOOSTER_SETTLING_MS) {
            noInterrupts(); v_rfid.bits = 0; v_rfid.code = 0; interrupts();
        }
        if (v_rfid.bits > 0 && (micros() - v_rfid.lastMicros > WIEGAND_TIMEOUT_US)) {
            WiegandData card;
            noInterrupts(); card = v_rfid; v_rfid.bits = 0; v_rfid.code = 0; interrupts();

            if (card.bits == 26) {
                uint32_t finalCode = (card.code >> 1) & 0xFFFFFF;
                Serial.printf("[RFID] 26-bit: %u\n", finalCode);

                if (finalCode == masterKey) {
                    editMode = !editMode;
                    stateTimer = millis(); activityTimer = millis();
                    addLog(editMode ? "מצב עריכה הופעל" : "מצב עריכה כובה");
                    Serial.printf("[MODE] Edit: %s\n", editMode ? "ON" : "OFF");
                } else if (editMode) {
                    if (!cardExists(finalCode)) {
                        cardAdd(finalCode);
                        char buf[48]; snprintf(buf, sizeof(buf), "כרטיס #%u נרשם", finalCode);
                        addLog(buf);
                        Serial.println("[EDIT] Card Registered");
                    } else {
                        cardRemove(finalCode);
                        char buf[48]; snprintf(buf, sizeof(buf), "כרטיס #%u נמחק", finalCode);
                        addLog(buf);
                        Serial.println("[EDIT] Card Deleted");
                    }
                    stateTimer = millis(); activityTimer = millis();
                } else {
                    if (cardExists(finalCode)) {
                        cardAdd(finalCode);
                        setSystemState(LOCK_OPEN);
                        char buf[48]; snprintf(buf, sizeof(buf), "נפתח: כרטיס #%u", finalCode);
                        addLog(buf);
                        activityTimer = millis();
                        rtcAddRecord(cachedTemp, cachedHum, getBatteryPct(), 2, 1, finalCode);
                        if (mqttClient.connected()) {
                            mqttPublishEvent("rfid_open", finalCode);
                            mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), false);
                        }
                    } else {
                        char buf[52]; snprintf(buf, sizeof(buf), "נדחה: כרטיס #%u לא מורשה", finalCode);
                        addLog(buf);
                        Serial.println("[RFID] Denied");
                        rtcAddRecord(cachedTemp, cachedHum, getBatteryPct(), 0, 3, finalCode);
                        if (mqttClient.connected()) mqttPublishEvent("denied", finalCode);
                    }
                }
            } else {
                Serial.printf("[SYS] Frame ignored: %d bits\n", card.bits);
            }
        }
        if (millis() - stateTimer > READER_TIMEOUT_MS) {
            addLog("קורא כובה (timeout)");
            setSystemState(IDLE);
        }
    }

    // Lock timer
    if (currentState == LOCK_OPEN && (millis() - stateTimer > LOCK_OPEN_MS)) {
        addLog("דלת ננעלה");
        setSystemState(IDLE);
        if (mqttClient.connected())
            mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), true);
    }

    // Deep sleep — מושבת

    yield();
}
