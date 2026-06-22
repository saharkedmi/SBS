#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPAsyncWebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <time.h>
#include <ESPmDNS.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <driver/rtc_io.h>
#include <PubSubClient.h>

// =====================================================================
// Pin Map — ESP32-S3-WROOM-1 N16R8
// GPIO 22–33 = internal flash/PSRAM — not on board headers
// RTC-capable GPIOs on S3: 0–21 only
// ADC1 on S3: GPIO 1–10 (ADC2 unusable with WiFi)
// GPIO 19/20 = USB D+/D- — never use as outputs
// =====================================================================
#define D0_PIN           5
#define D1_PIN           13
#define TOUCH_PIN        7    // RTC GPIO — EXT0 deep-sleep wake
#define BOOST_12V_EN_PIN 14
#define DHTPIN           16   // DS18B20 OneWire
#define BAT_ADC          4    // ADC1_CH3 — works with WiFi on
#define MOTOR_PWMA       8    // PWM speed — RTC GPIO, held LOW during sleep
#define MOTOR_AIN1       18   // Direction 1
#define MOTOR_AIN2       15   // Direction 2  (NOT 19 — USB D-)
#define MOTOR_STBY       17   // STBY: LOW=1µA sleep, HIGH=active

// Compile-time GPIO enum constants required by RTC/sleep API
#define MOTOR_PWMA_GPIO  GPIO_NUM_8
#define TOUCH_GPIO       GPIO_NUM_7

// =====================================================================
// Timing
// =====================================================================
#define LOCK_OPEN_MS        15000
#define LOCK_COOLDOWN_MS    2000
#define READER_TIMEOUT_MS   15000
#define WIEGAND_TIMEOUT_US  200000
#define BOOSTER_SETTLING_MS 80

#define MOTOR_MOVE_MS    1000   // default ms to complete lock/unlock stroke
#define MAX_PWM          150    // 60% duty — limits 5V supply to ~3V for N20

#define SLEEP_TIMEOUT_MS        300000
#define REPORT_INTERVAL_US      (10ULL * 60ULL * 1000000ULL)
#define WIFI_CONNECT_TIMEOUT_MS 10000

#define BAT_SAMPLE_MS    10000UL
#define BAT_SAMPLES      6
#define BAT_OUTLIER_V    0.3f
#define MQTT_PERIODIC_MS (10UL * 60UL * 1000UL)

// =====================================================================
// Network
// =====================================================================
const char* ssid         = "Kedmi";
const char* password     = "0504241190";
const char* www_username = "admin";
const char* www_password = "12345678";
const char* api_token    = "sbs_secure_99";

const char* mqtt_server   = "10.0.0.30";
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
// RTC Memory — survives deep sleep (8 KB available)
// =====================================================================
#define RTC_BUFFER_SIZE 72

struct __attribute__((packed)) SensorRecord {
    uint32_t timestamp_s;
    float    temp;
    float    hum;
    uint8_t  battery_pct;
    uint8_t  lock_state;
    uint8_t  event_type;
    uint8_t  reserved;
    uint32_t card_code;
};

RTC_DATA_ATTR SensorRecord rtcBuffer[RTC_BUFFER_SIZE];
RTC_DATA_ATTR uint8_t      rtcHead   = 0;
RTC_DATA_ATTR uint8_t      rtcCount  = 0;
RTC_DATA_ATTR uint32_t     bootCount = 0;

// =====================================================================
// Graph History — 24h at 10-min intervals (144 × 7 bytes = 1008 B RTC)
// =====================================================================
#define GRAPH_BUF_SIZE 144

struct __attribute__((packed)) GraphPoint {
    uint32_t ts;
    int16_t  temp10;
    uint8_t  bat_pct;
};

RTC_DATA_ATTR GraphPoint graphBuf[GRAPH_BUF_SIZE];
RTC_DATA_ATTR uint8_t    graphHead  = 0;
RTC_DATA_ATTR uint8_t    graphCount = 0;
RTC_DATA_ATTR uint32_t   ntpEpoch   = 0;
RTC_DATA_ATTR uint32_t   ntpMillis  = 0;

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

uint32_t getEpoch() {
    if (ntpEpoch == 0) return (uint32_t)(millis() / 1000);
    if (millis() < ntpMillis) return ntpEpoch;
    return ntpEpoch + (uint32_t)((millis() - ntpMillis) / 1000);
}

void graphAddPoint(float temp, int bat) {
    GraphPoint p;
    p.ts      = getEpoch();
    p.temp10  = isnan(temp) ? -9990 : (int16_t)(temp * 10.0f);
    p.bat_pct = (uint8_t)constrain(bat, 0, 100);
    graphBuf[graphHead] = p;
    graphHead = (graphHead + 1) % GRAPH_BUF_SIZE;
    if (graphCount < GRAPH_BUF_SIZE) graphCount++;
}

int graphGetIdx(int pos) {
    if (graphCount < GRAPH_BUF_SIZE) return pos;
    return (graphHead + pos) % GRAPH_BUF_SIZE;
}

// =====================================================================
// State Machine
// =====================================================================
enum SystemState { IDLE, READER_ACTIVE, LOCK_OPEN };
enum WakeReason  { WAKE_BOOT, WAKE_TOUCH, WAKE_TIMER };

SystemState currentState = IDLE;
WakeReason  wakeReason   = WAKE_BOOT;

struct WiegandData {
    volatile uint32_t      code;
    volatile int           bits;
    volatile unsigned long lastMicros;
};
WiegandData v_rfid = {0, 0, 0};

unsigned long stateTimer       = 0;
unsigned long lastUnlockTime   = 0;
unsigned long boosterStartTime = 0;
unsigned long activityTimer    = 0;
bool     editMode  = false;
uint32_t masterKey = 10311717;

OneWire           oneWire(DHTPIN);
DallasTemperature ds18(&oneWire);
AsyncWebServer    server(80);
Preferences       prefs;
WiFiClient        wifiClient;
WiFiMulti         wifiMulti;
PubSubClient      mqttClient(wifiClient);

float         cachedTemp  = NAN;
float         cachedHum   = NAN;
unsigned long lastDHTRead = 0;

float         batSamples[BAT_SAMPLES] = {};
int           batSampleIdx    = 0;
float         batVAvg         = NAN;
int           batPctAvg       = -1;
unsigned long lastBatSampleMs = 0;
unsigned long lastMqttMs      = 0;

// Motor runtime config — loaded from Preferences, tunable via /calib
int           motorMoveMs     = MOTOR_MOVE_MS;
bool          motorDirSwapped = false;          // true → A=lock, B=unlock
volatile unsigned long motorStopAt = 0;

// =====================================================================
// Activity Log
// =====================================================================
#define LOG_SIZE 15
struct LogEntry { char msg[52]; unsigned long ts; };
LogEntry actLog[LOG_SIZE];
int logHead = 0, logCount = 0;

void addLog(const char* msg) {
    strlcpy(actLog[logHead].msg, msg, sizeof(actLog[0].msg));
    actLog[logHead].ts = (ntpEpoch > 0 && millis() >= ntpMillis)
        ? ntpEpoch + (uint32_t)((millis() - ntpMillis) / 1000)
        : (uint32_t)(millis() / 1000);
    logHead = (logHead + 1) % LOG_SIZE;
    if (logCount < LOG_SIZE) logCount++;
}

// =====================================================================
// Battery
// =====================================================================
struct BatteryInfo { int raw; float vbat; int pct; };

BatteryInfo getBatteryInfo() {
    BatteryInfo b;
    b.raw  = analogRead(BAT_ADC);
    b.vbat = (analogReadMilliVolts(BAT_ADC) / 1000.0f) * 2.0f;
    b.pct  = constrain((int)((b.vbat - 3.0f) / 1.2f * 100.0f), 0, 100);
    return b;
}

int getBatteryPct() { return getBatteryInfo().pct; }

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
// Cards
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
    mqttClient.publish(
        MQTT_PREFIX "/sensor/" MQTT_DEVICE_ID "/temperature/config",
        "{\"name\":\"SmartSafe Temperature\","
        "\"state_topic\":\"" MQTT_STATE_TOPIC "\","
        "\"value_template\":\"{{ value_json.temp }}\","
        "\"unit_of_measurement\":\"°C\","
        "\"device_class\":\"temperature\","
        "\"unique_id\":\"sbs_temp\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\",\"model\":\"ESP32-S3\"}}",
        true);
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
    mqttClient.publish(
        MQTT_PREFIX "/sensor/" MQTT_DEVICE_ID "/last_event/config",
        "{\"name\":\"SmartSafe Last Event\","
        "\"state_topic\":\"" MQTT_EVENT_TOPIC "\","
        "\"unique_id\":\"sbs_event\","
        "\"device\":{\"identifiers\":[\"smartsafe_pro\"],\"name\":\"SmartSafe Pro\"}}",
        true);
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
// Motor Control — TB6612FNG + N20 3V @ 5V supply
// MAX_PWM=150 caps duty at 60% → ~3V effective (protects motor coils)
// =====================================================================
void motorForward() {
    digitalWrite(MOTOR_STBY, HIGH);
    digitalWrite(MOTOR_AIN1, HIGH);
    digitalWrite(MOTOR_AIN2, LOW);
    analogWrite(MOTOR_PWMA, MAX_PWM);
}

void motorReverse() {
    digitalWrite(MOTOR_STBY, HIGH);
    digitalWrite(MOTOR_AIN1, LOW);
    digitalWrite(MOTOR_AIN2, HIGH);
    analogWrite(MOTOR_PWMA, MAX_PWM);
}

void motorStop() {
    analogWrite(MOTOR_PWMA, 0);
    digitalWrite(MOTOR_AIN1, LOW);
    digitalWrite(MOTOR_AIN2, LOW);
}

void motorStandby() {
    motorStop();
    digitalWrite(MOTOR_STBY, LOW);
}

void motorUnlock() { motorDirSwapped ? motorReverse() : motorForward(); }
void motorLock()   { motorDirSwapped ? motorForward() : motorReverse(); }

// =====================================================================
// HTML — Dashboard
// =====================================================================
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
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
.bat-span{grid-column:span 2}
.btn-ul{width:100%;padding:15px;background:linear-gradient(135deg,#6366f1,#8b5cf6);border:none;border-radius:12px;color:#fff;font-size:.98rem;font-weight:700;cursor:pointer;transition:all .2s;margin-bottom:10px;letter-spacing:.02em}
.btn-ul:hover{transform:translateY(-1px);box-shadow:0 8px 25px var(--ag)}
.btn-ul:active{transform:translateY(0)}
.btn-ul:disabled{opacity:.45;cursor:not-allowed;transform:none}
.nav{display:flex;gap:8px;margin-bottom:14px}
.nav a{flex:1;text-align:center;padding:9px 4px;background:var(--s1);border:1px solid var(--bd);border-radius:10px;color:var(--tm);font-size:.73rem;text-decoration:none;transition:border-color .2s}
.nav a:hover{border-color:var(--ac);color:var(--tx)}
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
  <div><h1>&#128274; SmartSafe Pro</h1><div class="sub" id="uptime">Loading...</div></div>
  <div class="badge">&#9679; Connected</div>
</div>
<div class="g2">
  <div class="card lock-card">
    <div class="lock-ico lk" id="lIco">&#128274;</div>
    <div><div class="lock-h2" id="lStat">Locked</div><div class="lock-p" id="lSub">State: Idle</div></div>
  </div>
</div>
<div class="g2">
  <div class="card"><div class="lbl">Temperature</div><div class="val" id="tVal">--</div><div class="sub2">&#176;C</div></div>
  <div class="card">
    <div class="lbl">WiFi</div>
    <div class="wifi gd" id="wBars"><span></span><span></span><span></span><span></span></div>
    <div class="sub2" id="wRssi">-- dBm</div>
  </div>
  <div class="card bat-span">
    <div class="lbl">Battery</div>
    <div class="val" id="bVal">--%</div>
    <div class="bat-bar"><div class="bat-fill hi" id="bBar" style="width:0%"></div></div>
    <div id="bDbg" style="font-family:monospace;font-size:.62rem;color:var(--yw);margin-top:5px;line-height:1.5">raw: --<br>-- V</div>
  </div>
</div>
<button class="btn-ul" id="ulBtn" onclick="doUnlock()">&#128275; Unlock Door</button>
<div class="nav">
  <a href="/monitor">&#128270; Monitor</a>
  <a href="/graph">&#128200; History</a>
  <a href="/calib">&#9881; Calibration</a>
</div>
<div class="sec">Authorized Cards</div>
<div class="lst" id="cList"><div class="empty"><span class="spin"></span></div></div>
<div class="sec">Activity Log</div>
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
    document.getElementById('lStat').textContent=lk?'Locked':'Unlocked';
    const stMap={IDLE:'Idle',READER_ACTIVE:'Reader Active',LOCK_OPEN:'Door Open'};
    document.getElementById('lSub').textContent='State: '+(stMap[d.state]||d.state);
    document.getElementById('tVal').textContent=(d.temp>-40)?d.temp.toFixed(1):'ERR';
    const b=Math.min(100,Math.max(0,d.battery));
    document.getElementById('bVal').textContent=b+'%';
    const bf=document.getElementById('bBar');bf.style.width=b+'%';
    bf.className='bat-fill '+(b>50?'hi':b>20?'md':'lo');
    if(d.bat_raw!==undefined)document.getElementById('bDbg').innerHTML='raw: '+d.bat_raw+'<br>'+parseFloat(d.bat_v).toFixed(3)+' V';
    document.getElementById('wRssi').textContent=d.rssi+' dBm';
    const wb=document.getElementById('wBars');
    wb.className='wifi '+(d.rssi>-60?'gd':d.rssi>-75?'md':'wk');
    const s=d.uptime,h=Math.floor(s/3600),m=Math.floor((s%3600)/60);
    document.getElementById('uptime').textContent='Uptime: '+(h?h+'h ':'')+m+'m';
  }catch(e){}
}
async function fetchCards(){
  try{
    const d=await(await fetch('/api/cards')).json();
    const el=document.getElementById('cList');
    if(!d.cards||!d.cards.length){el.innerHTML='<div class="empty">No cards registered</div>';return;}
    el.innerHTML=d.cards.map(c=>`<div class="li"><div><div>RFID Card</div><div class="id">#${c}</div></div><button class="btn-del" onclick="delCard('${c}')">Remove</button></div>`).join('');
  }catch(e){}
}
async function fetchLog(){
  try{
    const d=await(await fetch('/api/log')).json();
    const el=document.getElementById('lList');
    if(!d.log||!d.log.length){el.innerHTML='<div class="empty">No events</div>';return;}
    el.innerHTML=d.log.map(e=>{
      const c=e.msg.includes('Opened')||e.msg.includes('open')?'ok':e.msg.includes('Denied')||e.msg.includes('denied')||e.msg.includes('Invalid')?'er':'in';
      const t=e.ts>86400?new Date(e.ts*1000).toLocaleTimeString():('0'+Math.floor(e.ts/3600)).slice(-2)+':'+('0'+Math.floor((e.ts%3600)/60)).slice(-2)+':'+('0'+(e.ts%60)).slice(-2);
      return`<div class="log-it ${c}"><div class="log-t">${t}</div><div class="log-m">${e.msg}</div></div>`;
    }).join('');
  }catch(e){}
}
async function doUnlock(){
  if(cd)return;const btn=document.getElementById('ulBtn');btn.disabled=true;cd=true;
  try{
    const r=await fetch('/open?t='+TK);
    if(r.ok){toast('Door unlocked!','ok');setTimeout(()=>{fetchSt();fetchLog();},600);}
    else if(r.status===429)toast('Cooldown active...','er');
    else toast('Unlock failed','er');
  }catch(e){toast('Connection error','er');}
  setTimeout(()=>{btn.disabled=false;cd=false;},3000);
}
async function delCard(id){
  if(!confirm('Delete card #'+id+'?'))return;
  try{
    const r=await fetch('/api/cards/delete?id='+id,{method:'POST'});
    if(r.ok){toast('Card removed','ok');fetchCards();}else toast('Error','er');
  }catch(e){toast('Connection error','er');}
}
fetchSt();fetchCards();fetchLog();
setInterval(fetchSt,5000);setInterval(fetchLog,10000);setInterval(fetchCards,8000);
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// HTML — History Graph
// =====================================================================
const char GRAPH_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>History &#8212; SmartSafe</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--bd:#2a2a45;--ac:#6366f1;--gr:#10b981;--cy:#06b6d4;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:520px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:16px}
.hdr h1{font-size:1.15rem;font-weight:700}.hdr .sub{color:var(--tm);font-size:.68rem;margin-top:2px}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:16px;margin-bottom:12px}
.chart-lbl{font-size:.65rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin-bottom:10px}
svg{width:100%;display:block;overflow:visible}
.empty{padding:32px;text-align:center;color:var(--tm);font-size:.82rem}
.stats{display:flex;gap:20px;margin-top:8px;flex-wrap:wrap}
.stat{font-size:.72rem;color:var(--tm)}
.stat b{color:var(--tx);font-weight:600;margin-left:3px}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#128200; History</h1><div class="sub" id="subEl">Loading...</div></div>
  <a href="/" class="back">&#8592; Dashboard</a>
</div>
<div class="card">
  <div class="chart-lbl">Temperature (&#176;C)</div>
  <div id="tChart"><div class="empty">Loading...</div></div>
  <div class="stats" id="tStats"></div>
</div>
<div class="card">
  <div class="chart-lbl">Battery (%)</div>
  <div id="bChart"><div class="empty">Loading...</div></div>
  <div class="stats" id="bStats"></div>
</div>
<script>
const PL=34,PR=8,PT=8,PB=22,IH=100,W=460,IW=W-PL-PR;
function chart(pts,yLo,yHi,stroke,unit){
  if(!pts||pts.length<2)return'<div class="empty">Not enough data</div>';
  const n=pts.length;
  const xS=i=>(PL+i/(n-1)*IW).toFixed(1);
  const yS=v=>(PT+IH-(v-yLo)/(yHi-yLo||1)*IH).toFixed(1);
  const steps=4;let g='';
  for(let i=0;i<=steps;i++){
    const v=yLo+(yHi-yLo)/steps*i,y=(PT+IH-(v-yLo)/(yHi-yLo||1)*IH).toFixed(1);
    g+=`<line x1="${PL}" y1="${y}" x2="${W-PR}" y2="${y}" stroke="#2a2a45" stroke-width="1"/>`;
    g+=`<text x="${PL-4}" y="${(parseFloat(y)+3.5).toFixed(1)}" text-anchor="end" font-size="9" fill="#64748b">${unit==='%'?v.toFixed(0):v.toFixed(1)}</text>`;
  }
  const tStep=Math.max(1,Math.floor(n/5));
  for(let i=0;i<n;i+=tStep){
    const p=pts[i],x=xS(i);
    const lbl=p.ts>86400?(()=>{const d=new Date(p.ts*1000);return d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0');})():(Math.floor(p.ts/3600)+'h');
    g+=`<text x="${x}" y="${PT+IH+14}" text-anchor="middle" font-size="9" fill="#64748b">${lbl}</text>`;
  }
  let area=`M${xS(0)},${yS(pts[0].v)}`,line=`M${xS(0)},${yS(pts[0].v)}`;
  for(let i=1;i<n;i++){area+=` L${xS(i)},${yS(pts[i].v)}`;line+=` L${xS(i)},${yS(pts[i].v)}`;}
  area+=` L${xS(n-1)},${(PT+IH).toFixed(1)} L${xS(0)},${(PT+IH).toFixed(1)} Z`;
  const lx=xS(n-1),ly=yS(pts[n-1].v);
  return`<svg viewBox="0 0 ${W} ${PT+IH+PB}" xmlns="http://www.w3.org/2000/svg">${g}<path d="${area}" fill="${stroke}" fill-opacity=".07"/><path d="${line}" fill="none" stroke="${stroke}" stroke-width="1.5" stroke-linejoin="round"/><circle cx="${lx}" cy="${ly}" r="3" fill="${stroke}"/></svg>`;
}
async function load(){
  try{
    const d=await(await fetch('/api/graph')).json();
    if(!d||!d.length){
      document.getElementById('tChart').innerHTML='<div class="empty">No data yet &#8212; recorded every 10 min</div>';
      document.getElementById('bChart').innerHTML='<div class="empty">No data yet</div>';
      document.getElementById('subEl').textContent='0 points';return;
    }
    document.getElementById('subEl').textContent=d.length+' point'+(d.length===1?'':'s')+' · up to 24h';
    const tp=d.filter(p=>p.temp!==null).map(p=>({ts:p.ts,v:p.temp}));
    if(tp.length>=2){
      const vs=tp.map(p=>p.v),mn=Math.min(...vs),mx=Math.max(...vs),pad=Math.max(.5,(mx-mn)*.15);
      document.getElementById('tChart').innerHTML=chart(tp,mn-pad,mx+pad,'#06b6d4','C');
      const last=tp[tp.length-1];
      document.getElementById('tStats').innerHTML=`<div class="stat">Now<b>${last.v.toFixed(1)}&#176;C</b></div><div class="stat">Min<b>${mn.toFixed(1)}&#176;C</b></div><div class="stat">Max<b>${mx.toFixed(1)}&#176;C</b></div>`;
    }else document.getElementById('tChart').innerHTML='<div class="empty">Not enough data</div>';
    const bp=d.map(p=>({ts:p.ts,v:p.bat}));
    const bvs=bp.map(p=>p.v),bmn=Math.max(0,Math.min(...bvs)-5),bmx=Math.min(100,Math.max(...bvs)+5);
    document.getElementById('bChart').innerHTML=chart(bp,bmn,bmx,'#10b981','%');
    const blast=bp[bp.length-1];
    document.getElementById('bStats').innerHTML=`<div class="stat">Now<b>${blast.v}%</b></div><div class="stat">Min<b>${Math.min(...bvs)}%</b></div><div class="stat">Max<b>${Math.max(...bvs)}%</b></div>`;
  }catch(e){
    document.getElementById('tChart').innerHTML='<div class="empty">Failed to load</div>';
    document.getElementById('bChart').innerHTML='<div class="empty">Failed to load</div>';
  }
}
load();setInterval(load,60000);
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// HTML — Monitor
// =====================================================================
const char MONITOR_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Monitor &#8212; SmartSafe</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--bd:#2a2a45;--ac:#6366f1;--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--cy:#06b6d4;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:520px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:16px}
.hdr h1{font-size:1.15rem;font-weight:700}.hdr .sub{color:var(--tm);font-size:.68rem;margin-top:2px}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.sec{font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin:14px 0 6px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:14px;margin-bottom:8px}
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
  <a href="/" class="back">&#8592; Dashboard</a>
</div>
<div class="sec">System State</div>
<div class="card" style="display:flex;align-items:center;gap:14px">
  <span id="stBadge" class="badge idle">IDLE</span>
  <span id="stDesc" style="color:var(--tm);font-size:.82rem">Idle</span>
</div>
<div class="sec">RFID Scan</div>
<div class="card">
  <div class="lbl" style="display:flex;align-items:center;gap:6px">Last scanned ID <span class="dot off" id="scanDot"></span></div>
  <div class="code-big" id="rfidCode">---</div>
  <div class="bits-sub" id="rfidBits">Not active</div>
</div>
<div class="sec">DS18B20 &#8212; Temperature (OneWire, GPIO16)</div>
<div class="card">
  <div class="lbl">Temperature</div>
  <div class="val" id="tRaw">--</div>
  <div style="color:var(--tm);font-size:.68rem;margin-top:3px">&#176;C</div>
  <div class="row" style="margin-top:8px"><span style="color:var(--tm)">Last read age</span><span class="mono" id="dhtAge">--</span></div>
</div>
<div class="sec">Battery &#8212; Raw &amp; Computed</div>
<div class="card">
  <div class="lbl">Live ADC reading</div>
  <div class="row"><span>ADC Raw (0&#8211;4095)</span><span class="mono" style="color:var(--yw)" id="batRaw">--</span></div>
  <div class="row"><span>Computed voltage</span><span class="mono" style="color:var(--cy)" id="batV">--</span></div>
  <div class="row"><span>Battery %</span><span class="mono" id="batPct">--</span></div>
</div>
<div class="card">
  <div class="lbl">Sample window &#8212; 6 &#215; 10 seconds</div>
  <div id="slotsEl"></div>
  <div class="avg-row">
    <span class="avg-v" id="avgV">--</span>
    <span class="avg-pct" id="avgPct">Waiting for samples...</span>
  </div>
</div>
<div class="sec">Stored Keys</div>
<div class="card" id="cardsEl"><div class="empty">Loading...</div></div>
<script>
const stMap={
  IDLE:{label:'IDLE',cls:'idle',desc:'Idle'},
  READER_ACTIVE:{label:'READER ACTIVE',cls:'active',desc:'RFID reader active'},
  LOCK_OPEN:{label:'LOCK OPEN',cls:'open',desc:'Door open'}
};
function fmtUp(s){const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;return(h?h+'h ':'')+m+'m '+sec+'s';}
async function refresh(){
  try{
    const d=await(await fetch('/api/monitor')).json();
    document.getElementById('upEl').textContent='Uptime: '+fmtUp(d.uptime);
    const sm=stMap[d.state]||{label:d.state,cls:'idle',desc:''};
    const b=document.getElementById('stBadge');b.textContent=sm.label;b.className='badge '+sm.cls;
    document.getElementById('stDesc').textContent=sm.desc;
    const sc=d.rfid.scanning;
    document.getElementById('scanDot').className='dot '+(sc?'on':'off');
    document.getElementById('rfidCode').textContent=(sc&&d.rfid.bits>0)?String(d.rfid.code):'---';
    document.getElementById('rfidBits').textContent=(sc&&d.rfid.bits>0)?(d.rfid.bits+' bits'):'Not active';
    document.getElementById('tRaw').textContent=d.dht.temp!==null?d.dht.temp.toFixed(2):'ERR';
    document.getElementById('dhtAge').textContent=(d.dht.age_ms/1000).toFixed(1)+'s';
    document.getElementById('batRaw').textContent=d.bat_live.raw;
    document.getElementById('batV').textContent=d.bat_live.v.toFixed(3)+' V';
    document.getElementById('batPct').textContent=d.bat_live.pct+'%';
    const sl=d.bat_samples,idx=d.bat_sample_idx,ready=d.bat_avg_ready;
    let html='';
    for(let i=0;i<6;i++){
      const v=sl[i],has=v>0.5,isNext=i===idx;
      const pct=has?Math.max(0,Math.min(100,Math.round((v-3.0)/1.2*100))):null;
      html+=`<div class="slot"><span class="slot-n">${i+1}</span><span class="slot-v${has?'':' empty'}">${has?v.toFixed(3)+' V':'---'}</span><span class="slot-pct">${pct!==null?pct+'%':''}</span><span class="slot-tag${isNext?' next':has?' ok':''}">${isNext?'&#8592; next':has?'&#10003;':''}</span></div>`;
    }
    document.getElementById('slotsEl').innerHTML=html;
    if(ready&&d.bat_avg_v!=null){
      document.getElementById('avgV').textContent=d.bat_avg_v.toFixed(3)+' V';
      document.getElementById('avgPct').textContent='Filtered avg · '+d.bat_avg_pct+'%';
    }else{
      document.getElementById('avgV').textContent='--';
      document.getElementById('avgPct').textContent='Waiting ('+(ready?6:idx)+'/6 samples)...';
    }
    const cards=d.cards,cel=document.getElementById('cardsEl');
    if(!cards||!cards.length)cel.innerHTML='<div class="empty">No stored keys</div>';
    else cel.innerHTML=cards.map((c,i)=>`<div class="card-item"><span>Key ${i+1}</span><span class="card-id">#${c}</span></div>`).join('');
  }catch(e){console.error(e);}
}
refresh();setInterval(refresh,1000);
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// HTML — Motor Calibration
// =====================================================================
const char CALIB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Motor Calibration &#8212; SmartSafe</title>
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
  <div><h1>&#9881; Motor Calibration</h1></div>
  <a href="/" class="back">&#8592; Dashboard</a>
</div>

<div class="step-num">Step 1</div>
<div class="sec">Test directions</div>
<div class="card">
  <div class="lbl">Test duration</div>
  <div class="ms-big" id="msDsp">1000ms</div>
  <input type="range" id="msSlider" min="100" max="2000" step="50" value="1000"
         oninput="document.getElementById('msDsp').textContent=this.value+'ms'">
  <div class="row">
    <button class="btn ba" onclick="runDir('fwd')">&#9654; Direction A (Forward)</button>
    <button class="btn bb" onclick="runDir('rev')">&#9654; Direction B (Reverse)</button>
    <button class="btn bstop" onclick="doStop()" title="Stop">&#9632;</button>
  </div>
</div>

<div class="step-num">Step 2</div>
<div class="sec">Set open direction</div>
<div class="card">
  <div class="lbl">Select which direction opens the lock</div>
  <div class="row">
    <button class="btn ba" onclick="setDir('A')">A = Open &#10003;</button>
    <button class="btn bb" onclick="setDir('B')">B = Open &#10003;</button>
  </div>
  <div id="dirChip"><span class="chip chip-none">Not set yet</span></div>
</div>

<div class="step-num">Step 3</div>
<div class="sec">Tune duration</div>
<div class="card">
  <div class="lbl">ms until latch reaches end — increase if incomplete, decrease if motor stalls under pressure</div>
  <input type="number" id="saveMs" min="100" max="3000" step="50" value="1000">
  <div class="row">
    <button class="btn bopen" id="btnOpen" onclick="testAction('open')" disabled>&#9654; Test Open</button>
    <button class="btn bstop" id="btnClose" onclick="testAction('close')" disabled style="flex:1">&#9654; Test Close</button>
  </div>
</div>

<div class="step-num">Step 4</div>
<div class="sec">Save to flash</div>
<div class="card">
  <button class="bsave" id="btnSave" onclick="save()" disabled>&#128190; Save Calibration</button>
  <div id="saveMsg"></div>
</div>

<script>
let unlockDir=null;
function ms(){return parseInt(document.getElementById('msSlider').value);}
function sms(){return parseInt(document.getElementById('saveMs').value);}
async function api(p){try{const r=await fetch('/api/calib?'+p);return await r.json();}catch(e){return{ok:false};}}
function runDir(d){api('cmd='+d+'&ms='+ms());}
function doStop(){api('cmd=stop');}
function setDir(d){
  unlockDir=d;
  document.getElementById('dirChip').innerHTML='<span class="chip chip-ok">Direction '+d+' = Open &#10003;</span>';
  document.getElementById('btnOpen').disabled=false;
  document.getElementById('btnClose').disabled=false;
  document.getElementById('btnSave').disabled=false;
}
function testAction(a){
  const isFwd=(a==='open')?(unlockDir==='A'):(unlockDir==='B');
  api('cmd='+(isFwd?'fwd':'rev')+'&ms='+sms());
}
async function save(){
  const r=await api('cmd=save&dir='+unlockDir+'&ms='+sms());
  const el=document.getElementById('saveMsg');
  el.innerHTML=r&&r.ok
    ?'<div class="msg msg-ok">&#10003; Saved! Direction '+unlockDir+' opens &middot; '+sms()+'ms</div>'
    :'<div class="msg msg-er">&#10007; Save failed</div>';
}
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// ISR — Wiegand RFID
// =====================================================================
void IRAM_ATTR rfid_isr_d0() {
    if (currentState != READER_ACTIVE) return;
    v_rfid.code = (v_rfid.code << 1);      // bit 0
    v_rfid.bits++;
    v_rfid.lastMicros = micros();
}

void IRAM_ATTR rfid_isr_d1() {
    if (currentState != READER_ACTIVE) return;
    v_rfid.code = (v_rfid.code << 1) | 1;  // bit 1
    v_rfid.bits++;
    v_rfid.lastMicros = micros();
}

// =====================================================================
// FSM
// =====================================================================
void setSystemState(SystemState newState) {
    currentState  = newState;
    stateTimer    = millis();
    activityTimer = millis();

    if (newState == IDLE) {
        digitalWrite(BOOST_12V_EN_PIN, LOW);
        motorLock();
        motorStopAt = millis() + motorMoveMs;
        editMode = false;
        Serial.println("[FSM] IDLE — locking");
    } else if (newState == READER_ACTIVE) {
        digitalWrite(BOOST_12V_EN_PIN, HIGH);
        boosterStartTime = millis();
        noInterrupts(); v_rfid.bits = 0; v_rfid.code = 0; interrupts();
        addLog("RFID reader activated");
        Serial.println("[FSM] READER_ACTIVE");
    } else if (newState == LOCK_OPEN) {
        motorUnlock();
        motorStopAt = millis() + motorMoveMs;
        digitalWrite(BOOST_12V_EN_PIN, LOW);
        lastUnlockTime = millis();
        Serial.println("[FSM] LOCK_OPEN — unlocking");
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
                addLog("Opened via web");
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
            addLog("Invalid token attempt");
            req->send(403, "text/plain", "Invalid Token");
        }
    });

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        BatteryInfo bat = getBatteryInfo();
        int   dispPct  = (batPctAvg >= 0)  ? batPctAvg : bat.pct;
        float dispVolt = !isnan(batVAvg)   ? batVAvg   : bat.vbat;
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
        char buf[48]; snprintf(buf, sizeof(buf), "Card #%u removed (web)", code);
        addLog(buf);
        req->send(200, "text/plain", "OK");
    });

    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        String json = "{\"log\":[";
        int cnt = min(logCount, LOG_SIZE); bool first = true;
        for (int i = cnt - 1; i >= 0; i--) {
            int idx = ((logHead - 1 - i) % LOG_SIZE + LOG_SIZE) % LOG_SIZE;
            if (!first) json += ",";
            json += "{\"ts\":";    json += actLog[idx].ts;
            json += ",\"msg\":\""; json += actLog[idx].msg; json += "\"}";
            first = false;
        }
        json += "]}";
        req->send(200, "application/json", json);
    });

    server.on("/graph", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        req->send_P(200, "text/html", GRAPH_PAGE);
    });

    server.on("/api/graph", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        String json = "[";
        for (int i = 0; i < graphCount; i++) {
            int idx = graphGetIdx(i);
            GraphPoint& p = graphBuf[idx];
            if (i > 0) json += ",";
            json += "{\"ts\":";   json += p.ts;
            json += ",\"temp\":";
            if (p.temp10 == -9990) json += "null";
            else json += String(p.temp10 / 10.0f, 1);
            json += ",\"bat\":";  json += p.bat_pct;
            json += "}";
        }
        json += "]";
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

        if (cmd == "fwd") {
            int ms = req->hasParam("ms") ? constrain(req->getParam("ms")->value().toInt(), 50, 3000) : 600;
            motorForward();
            motorStopAt = millis() + ms;
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "rev") {
            int ms = req->hasParam("ms") ? constrain(req->getParam("ms")->value().toInt(), 50, 3000) : 600;
            motorReverse();
            motorStopAt = millis() + ms;
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "stop") {
            motorStopAt = 0;
            motorStop();
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "save") {
            String dir  = req->hasParam("dir") ? req->getParam("dir")->value() : "A";
            int    ms   = req->hasParam("ms")  ? constrain(req->getParam("ms")->value().toInt(), 50, 3000) : MOTOR_MOVE_MS;
            motorMoveMs     = ms;
            motorDirSwapped = (dir == "B");
            prefs.putInt ("m_ms",  ms);
            prefs.putBool("m_dir", motorDirSwapped);
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

        json += ",\"dht\":{\"temp\":";
        json += isnan(cachedTemp) ? "null" : String(cachedTemp, 1);
        json += ",\"hum\":null";
        json += ",\"age_ms\":"; json += (millis() - lastDHTRead);
        json += "}";

        json += ",\"bat_live\":{\"raw\":"; json += bat.raw;
        json += ",\"v\":";   json += String(bat.vbat, 3);
        json += ",\"pct\":"; json += bat.pct;
        json += "}";

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
    Serial.printf("[SYS] Boot #%u  (ESP32-S3-WROOM-1 N16R8)\n", bootCount);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if      (cause == ESP_SLEEP_WAKEUP_EXT0)  wakeReason = WAKE_TOUCH;
    else if (cause == ESP_SLEEP_WAKEUP_TIMER) wakeReason = WAKE_TIMER;
    else                                       wakeReason = WAKE_BOOT;

    Serial.printf("[SYS] Wake: %s\n",
        wakeReason == WAKE_TOUCH ? "TOUCH" :
        wakeReason == WAKE_TIMER ? "TIMER" : "BOOT");

    // ── Timer Wake: read sensors, report, go back to sleep ───────────
    if (wakeReason == WAKE_TIMER) {
        ds18.begin();
        ds18.requestTemperatures();
        delay(750);
        float t = ds18.getTempCByIndex(0);
        if (t < -100.0f) t = NAN;
        int bat = getBatteryPct();
        Serial.printf("[TIMER] T=%.1f BAT=%d\n", isnan(t) ? -99.0f : t, bat);

        WiFi.mode(WIFI_STA);
        wifiMulti.addAP(ssid, password);
        unsigned long wStart = millis();
        while (wifiMulti.run() != WL_CONNECTED && millis() - wStart < WIFI_CONNECT_TIMEOUT_MS)
            delay(200);

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
            configTime(2 * 3600, 3600, "pool.ntp.org", "time.cloudflare.com");
            time_t now = 0;
            for (int i = 0; i < 25 && now < 1000000UL; i++) { delay(200); time(&now); }
            if (now > 1000000UL) {
                ntpEpoch  = (uint32_t)now;
                ntpMillis = millis();
                Serial.printf("[NTP] Synced: %lu\n", (unsigned long)now);
            }
            graphAddPoint(t, bat);
            if (connectMQTT()) {
                mqttPublishDiscovery();
                mqttFlushBuffer();
                mqttPublishState(t, NAN, bat, true);
                for (int i = 0; i < 20; i++) { mqttClient.loop(); delay(100); }
                mqttClient.disconnect();
            } else {
                rtcAddRecord(t, NAN, (uint8_t)bat, 0, 0);
            }
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
        } else {
            if (ntpEpoch > 0) ntpEpoch += (uint32_t)(REPORT_INTERVAL_US / 1000000ULL);
            graphAddPoint(t, bat);
            rtcAddRecord(t, NAN, (uint8_t)bat, 0, 0);
            Serial.println("[WiFi] Timeout — data buffered");
        }

        Serial.println("[SYS] Timer wake done — returning to deep sleep");
        rtc_gpio_init(MOTOR_PWMA_GPIO);
        rtc_gpio_set_direction(MOTOR_PWMA_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(MOTOR_PWMA_GPIO, 0);
        rtc_gpio_hold_en(MOTOR_PWMA_GPIO);
        esp_sleep_enable_ext0_wakeup(TOUCH_GPIO, 1);
        esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_US);
        esp_deep_sleep_start();
    }

    // ── Touch Wake / Boot: normal operation ─────────────────────────
    pinMode(TOUCH_PIN,        INPUT_PULLDOWN);
    pinMode(BOOST_12V_EN_PIN, OUTPUT);
    pinMode(D0_PIN,           INPUT_PULLUP);
    pinMode(D1_PIN,           INPUT_PULLUP);
    digitalWrite(BOOST_12V_EN_PIN, LOW);

    // Motor pins
    pinMode(MOTOR_PWMA, OUTPUT);
    pinMode(MOTOR_AIN1, OUTPUT);
    pinMode(MOTOR_AIN2, OUTPUT);
    pinMode(MOTOR_STBY, OUTPUT);
    motorStandby();

    attachInterrupt(digitalPinToInterrupt(D0_PIN), rfid_isr_d0, FALLING);
    attachInterrupt(digitalPinToInterrupt(D1_PIN), rfid_isr_d1, FALLING);

    prefs.begin("safe-app", false);
    motorMoveMs     = prefs.getInt ("m_ms",  MOTOR_MOVE_MS);
    motorDirSwapped = prefs.getBool("m_dir", false);
    Serial.printf("[MOTOR] moveMs=%d  dirSwapped=%d\n", motorMoveMs, motorDirSwapped);

    // Release RTC hold on MOTOR_PWMA after touch wake
    if (wakeReason == WAKE_TOUCH) {
        rtc_gpio_hold_dis(MOTOR_PWMA_GPIO);
        rtc_gpio_deinit(MOTOR_PWMA_GPIO);
    }

    ds18.begin();
    ds18.setResolution(12);
    ds18.setWaitForConversion(false);
    ds18.requestTemperatures();
    delay(800);
    {
        float _t = ds18.getTempCByIndex(0);
        if (_t > -100.0f) {
            cachedTemp = _t;
            Serial.printf("[DS18B20] Init: %.2f°C on GPIO%d\n", _t, DHTPIN);
        } else {
            Serial.printf("[DS18B20] Init FAILED — check GPIO%d + 4.7kΩ to 3.3V\n", DHTPIN);
        }
    }
    lastDHTRead = millis();

    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(ssid, password);
    for (int i = 0; i < 4 && wifiMulti.run() != WL_CONNECTED; i++) delay(500);

    if (WiFi.status() == WL_CONNECTED) {
        MDNS.begin("smartsafe");
        Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.println("[WiFi] URL: http://smartsafe.local");

        configTime(2 * 3600, 3600, "pool.ntp.org", "time.cloudflare.com");
        time_t now = 0;
        for (int i = 0; i < 25 && now < 1000000UL; i++) { delay(200); time(&now); }
        if (now > 1000000UL) {
            ntpEpoch  = (uint32_t)now;
            ntpMillis = millis();
            Serial.printf("[NTP] Synced: %lu\n", (unsigned long)now);
        } else {
            Serial.println("[NTP] Sync failed");
        }

        if (connectMQTT()) {
            mqttPublishDiscovery();
            mqttFlushBuffer();
            mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), true);
        }
    } else {
        Serial.println("[WiFi] Connection FAILED");
    }

    setupServerRoutes();
    server.begin();

    setSystemState(IDLE);
    activityTimer = millis();

    if (wakeReason == WAKE_TOUCH) {
        addLog("Touch wake — reader activated");
        Serial.println("[SYS] Wake from touch — activating reader");
    } else {
        addLog("System started — reader active");
        Serial.println("[SYS] Boot — activating reader");
    }
    setSystemState(READER_ACTIVE);
    activityTimer = millis();
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
    // Non-blocking motor stop (used by calib, lock, unlock)
    if (motorStopAt > 0 && millis() >= motorStopAt) {
        motorStop();
        motorStopAt = 0;
    }

    // MQTT keepalive
    if (mqttClient.connected()) mqttClient.loop();

    // WiFi + MQTT reconnect every 60s when idle
    static unsigned long lastNetCheck = 0;
    if (currentState != READER_ACTIVE && millis() - lastNetCheck >= 60000) {
        lastNetCheck = millis();
        if (WiFi.status() != WL_CONNECTED) wifiMulti.run(2000);
        if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
            if (connectMQTT()) { mqttPublishDiscovery(); mqttFlushBuffer(); }
        }
    }

    // DS18B20 — request every 5s, read result after 750ms (non-blocking)
    static bool ds18Pending = false;
    static unsigned long ds18ReqAt = 0;
    if (!ds18Pending && millis() - lastDHTRead >= 5000) {
        ds18.requestTemperatures();
        ds18ReqAt = millis();
        ds18Pending = true;
    }
    if (ds18Pending && millis() - ds18ReqAt >= 750) {
        float t = ds18.getTempCByIndex(0);
        ds18Pending = false;
        lastDHTRead = millis();
        if (t > -100.0f) {
            cachedTemp = t;
            Serial.printf("[DS18B20] %.2f°C\n", t);
        } else {
            Serial.println("[DS18B20] Read FAILED");
        }
    }

    // Battery sampling every 10s; filtered average after 6 samples
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

    // MQTT periodic (every 10 min, only when averaged data is ready)
    if (batPctAvg >= 0 && mqttClient.connected() &&
        millis() - lastMqttMs >= MQTT_PERIODIC_MS) {
        lastMqttMs = millis();
        mqttPublishState(cachedTemp, cachedHum, batPctAvg, currentState != LOCK_OPEN);
        graphAddPoint(cachedTemp, batPctAvg);
    }

    // Touch button
    static bool lastTouch = LOW;
    bool touch = digitalRead(TOUCH_PIN);
    if (touch == HIGH && lastTouch == LOW) {
        activityTimer = millis();
        if (currentState == IDLE)               setSystemState(READER_ACTIVE);
        else if (currentState == READER_ACTIVE) stateTimer = millis();
    }
    lastTouch = touch;

    // RFID Wiegand processing
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
                    addLog(editMode ? "Edit mode ON" : "Edit mode OFF");
                } else if (editMode) {
                    if (!cardExists(finalCode)) {
                        cardAdd(finalCode);
                        char buf[48]; snprintf(buf, sizeof(buf), "Card #%u registered", finalCode);
                        addLog(buf);
                    } else {
                        cardRemove(finalCode);
                        char buf[48]; snprintf(buf, sizeof(buf), "Card #%u removed", finalCode);
                        addLog(buf);
                    }
                    stateTimer = millis(); activityTimer = millis();
                } else {
                    if (cardExists(finalCode)) {
                        setSystemState(LOCK_OPEN);
                        char buf[48]; snprintf(buf, sizeof(buf), "Opened: card #%u", finalCode);
                        addLog(buf);
                        activityTimer = millis();
                        rtcAddRecord(cachedTemp, cachedHum, getBatteryPct(), 2, 1, finalCode);
                        if (mqttClient.connected()) {
                            mqttPublishEvent("rfid_open", finalCode);
                            mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), false);
                        }
                    } else {
                        char buf[52]; snprintf(buf, sizeof(buf), "Denied: card #%u", finalCode);
                        addLog(buf);
                        rtcAddRecord(cachedTemp, cachedHum, getBatteryPct(), 0, 3, finalCode);
                        if (mqttClient.connected()) {
                            mqttPublishEvent("denied", finalCode);
                            mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), true);
                        }
                    }
                }
            } else {
                Serial.printf("[SYS] Frame ignored: %d bits\n", card.bits);
            }
        }
        if (millis() - stateTimer > READER_TIMEOUT_MS) {
            addLog("Reader timeout");
            setSystemState(IDLE);
        }
    }

    // Auto-lock after LOCK_OPEN_MS
    if (currentState == LOCK_OPEN && millis() - stateTimer > LOCK_OPEN_MS) {
        addLog("Door locked");
        setSystemState(IDLE);
        if (mqttClient.connected())
            mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), true);
    }

    // Idle timeout → deep sleep
    if (currentState == IDLE && millis() - activityTimer > SLEEP_TIMEOUT_MS) {
        addLog("Idle timeout — going to sleep");
        Serial.printf("[SYS] No activity for %lus — entering deep sleep\n",
                      (millis() - activityTimer) / 1000);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        motorStandby();
        rtc_gpio_init(MOTOR_PWMA_GPIO);
        rtc_gpio_set_direction(MOTOR_PWMA_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(MOTOR_PWMA_GPIO, 0);
        rtc_gpio_hold_en(MOTOR_PWMA_GPIO);
        esp_sleep_enable_ext0_wakeup(TOUCH_GPIO, 1);
        esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_US);
        esp_deep_sleep_start();
    }

    yield();
}
