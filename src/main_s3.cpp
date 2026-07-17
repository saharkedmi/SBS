#include <Arduino.h>
#include <WiFi.h>
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
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <Update.h>
#include <DNSServer.h>
#include <esp_ota_ops.h>
#include <driver/gpio.h>

#define FW_VERSION "2.2.4"

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
#define SERVO_PIN        8    // Servo PWM signal — RTC GPIO, held LOW during sleep
#define LED_PIN          6    // WS2812B data
#define LED_COUNT        12

// Compile-time GPIO enum constants required by RTC/sleep API
#define SERVO_PIN_GPIO   GPIO_NUM_8
#define TOUCH_GPIO       GPIO_NUM_7

// =====================================================================
// Timing
// =====================================================================
#define LOCK_HOLD_MS        10000  // hold time after servo reaches open position
#define LOCK_COOLDOWN_MS    2000
#define READER_TIMEOUT_MS   15000
#define WIEGAND_TIMEOUT_US  200000
#define BOOSTER_SETTLING_MS 80
#define SERVO_DETACH_MS     1500   // ms after write before detaching servo

#define SLEEP_TIMEOUT_MS        300000
#define REPORT_INTERVAL_US      (10ULL * 60ULL * 1000000ULL)
#define WIFI_CONNECT_TIMEOUT_MS 20000  // total budget, blocking (timer wake only)
#define WIFI_TRY_MS             8000   // per-network attempt, non-blocking
#define WIFI_RETRY_MS           20000  // retry period after all networks failed
#define AP_SSID                 "SBS"
#define AP_TIMEOUT_MS           120000 // close AP if no client joined within 2 min
#define MQTT_RETRY_MS           30000
#define MQTT_FLUSH_SPACING_MS   1500   // gap between buffered-record publishes on reconnect,
                                        // so HA's history doesn't compress them into one instant

#define BAT_SAMPLE_MS    10000UL
#define BAT_SAMPLES      6
#define BAT_OUTLIER_V    0.3f
#define BAT_MIN_V        2.6f   // below any real single-cell LiPo
#define BAT_MAX_V        4.35f  // above any real single-cell LiPo (incl. charge overshoot)
#define TEMP_MIN_C       -20.0f // below any plausible indoor/ambient reading for this device
#define TEMP_MAX_C        60.0f // above any plausible indoor/ambient reading for this device
#define MQTT_PERIODIC_MS (10UL * 60UL * 1000UL)

// =====================================================================
// Network
// =====================================================================
// WiFi seed — copied into the NVS network list on first boot only.
// Real values live in NVS: configure via the /settings and /wifi pages
// (or the SBS setup AP on a fresh device).
const char* ssid     = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";

// Runtime settings — first-boot defaults; overridden from NVS via /settings
char www_username[24] = "admin";
char www_password[32] = "Your_Web_Password";
char api_token[33]    = "Your_API_Token";
char mqtt_server[40]  = "Your_MQTT_Server";
int  mqtt_port        = 1883;
char mqtt_user[24]    = "Your_MQTT_User";
char mqtt_pass[32]    = "Your_MQTT_Pass";

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
    rec.temp        = t;  // may be NaN — serialized as JSON null, never a fake number
    rec.hum         = h;
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

Preferences       prefs;

// =====================================================================
// Persistent Log — NVS-backed, survives power-off (144 × 8 B = 1152 B)
// evt: 0=sensor, 1=unlock, 2=lock, 3=denied, 4=boot
// =====================================================================
#define PLOG_MAX 144

struct __attribute__((packed)) PLogRecord {
    uint32_t ts;
    int16_t  temp10;
    uint8_t  bat;
    uint8_t  evt;
};

PLogRecord plogBuf[PLOG_MAX];
int plogHead  = 0;
int plogCount = 0;

// Records written before NTP sync carry relative timestamps; remember
// them so they can be corrected retroactively once the clock is known.
uint8_t plogUnsynced[8];
int     plogUnsyncedCnt = 0;

void plogLoad() {
    if (prefs.getBytesLength("pl_d") == sizeof(plogBuf)) {
        prefs.getBytes("pl_d", plogBuf, sizeof(plogBuf));
        plogHead  = prefs.getInt("pl_h", 0);
        plogCount = prefs.getInt("pl_n", 0);
        if (plogHead < 0 || plogHead >= PLOG_MAX) plogHead = 0;
        if (plogCount < 0 || plogCount > PLOG_MAX) plogCount = 0;
    }
}

void plogSave() {
    prefs.putBytes("pl_d", plogBuf, sizeof(plogBuf));
    prefs.putInt("pl_h", plogHead);
    prefs.putInt("pl_n", plogCount);
}

void plogAdd(float temp, uint8_t bat, uint8_t evt) {
    PLogRecord r;
    r.ts     = getEpoch();
    r.temp10 = isnan(temp) ? (int16_t)-9990 : (int16_t)(temp * 10.0f);
    r.bat    = bat;
    r.evt    = evt;
    if (ntpEpoch == 0 && plogUnsyncedCnt < (int)sizeof(plogUnsynced))
        plogUnsynced[plogUnsyncedCnt++] = (uint8_t)plogHead;
    plogBuf[plogHead] = r;
    plogHead = (plogHead + 1) % PLOG_MAX;
    if (plogCount < PLOG_MAX) plogCount++;
    plogSave();
}

// Called once when NTP first syncs — convert this session's relative
// timestamps to absolute epoch
void plogFixTimestamps() {
    if (plogUnsyncedCnt == 0) return;
    uint32_t nowRel = millis() / 1000;
    for (int i = 0; i < plogUnsyncedCnt; i++) {
        PLogRecord& r = plogBuf[plogUnsynced[i]];
        if (r.ts < 1000000UL && r.ts <= nowRel)
            r.ts = ntpEpoch - (nowRel - r.ts);
    }
    plogUnsyncedCnt = 0;
    plogSave();
    Serial.println("[PLOG] Pre-sync timestamps corrected");
}

int plogGetIdx(int pos) {
    if (plogCount < PLOG_MAX) return pos;
    return (plogHead + pos) % PLOG_MAX;
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
WiFiClient        wifiClient;
PubSubClient      mqttClient(wifiClient);

// =====================================================================
// WiFi Manager — NVS multi-network store + non-blocking connect + AP
// =====================================================================
#define WIFI_MAX_NETS 5
struct WifiNet { char ssid[33]; char pass[65]; };
WifiNet wifiNets[WIFI_MAX_NETS] = {};
int     wifiNetCount = 0;

enum WifiMode { WM_OFF, WM_CONNECTING, WM_CONNECTED, WM_LOST };
WifiMode      wifiMode        = WM_OFF;
bool          wifiApFallback  = false;  // raise AP if every network fails
int           wifiTryIdx      = 0;      // attempt counter within one round
int           wifiCurIdx      = -1;     // index currently being tried
unsigned long wifiTryStart    = 0;
unsigned long lastWifiKick    = 0;
bool          apActive        = false;
unsigned long apStartMs       = 0;
int           apPrevClients   = 0;
DNSServer*    dnsServer       = nullptr;  // captive portal DNS, AP mode only
unsigned long lastApScanMs    = 0;        // proactive scan while AP is up
bool          apScanPending   = false;
unsigned long uiScanUntil     = 0;        // user-initiated scan in progress
bool          serverStarted   = false;
bool          mdnsStarted     = false;
bool          otaBusy         = false;
bool          fwPending       = false;  // new fw awaiting validation
bool          mqttEverConnected = false;
unsigned long lastMqttTryMs   = 0;

RTC_DATA_ATTR int8_t wifiLastGood = -1;  // last successful net — tried first

float         cachedTemp  = NAN;
float         cachedHum   = NAN;
unsigned long lastDHTRead = 0;

float         batSamples[BAT_SAMPLES] = {};
int           batSampleIdx    = 0;
float         batVAvg         = NAN;
int           batPctAvg       = -1;
unsigned long lastBatSampleMs = 0;
float         batCalFactor    = 1.0f;  // divider-tolerance correction, set via /calib
unsigned long lastMqttMs      = 0;
unsigned long lastPlogMs      = 0;

// Servo runtime config — loaded from Preferences, tunable via /calib
bool          motorDirSwapped = true;   // true → A(0°)=lock, B(180°)=unlock
unsigned long lockHoldStart   = 0;
unsigned long servoDetachAt   = 0;

Servo servo;

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

static float takeBatReading() {
    float v[3];
    for (int i = 0; i < 3; i++) {
        delayMicroseconds(500);
        v[i] = (analogReadMilliVolts(BAT_ADC) / 1000.0f) * 2.0f * batCalFactor;
    }
    if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
    if (v[1] > v[2]) { float t = v[1]; v[1] = v[2]; v[2] = t; }
    if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
    return v[1];
}

BatteryInfo getBatteryInfo() {
    BatteryInfo b;
    b.raw  = analogRead(BAT_ADC);
    b.vbat = (analogReadMilliVolts(BAT_ADC) / 1000.0f) * 2.0f * batCalFactor;
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
    char tempBuf[16], humBuf[16];
    if (isnan(temp)) strcpy(tempBuf, "null"); else snprintf(tempBuf, sizeof(tempBuf), "%.1f", temp);
    if (isnan(hum))  strcpy(humBuf,  "null"); else snprintf(humBuf,  sizeof(humBuf),  "%.0f", hum);
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"temp\":%s,\"hum\":%s,\"battery\":%d,\"locked\":%s,\"boot_count\":%u}",
        tempBuf, humBuf, bat, locked ? "true" : "false", bootCount);
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
        char tempBuf[16], humBuf[16];
        if (isnan(rec.temp)) strcpy(tempBuf, "null"); else snprintf(tempBuf, sizeof(tempBuf), "%.1f", rec.temp);
        if (isnan(rec.hum))  strcpy(humBuf,  "null"); else snprintf(humBuf,  sizeof(humBuf),  "%.0f", rec.hum);
        char payload[256];
        snprintf(payload, sizeof(payload),
            "{\"temp\":%s,\"hum\":%s,\"battery\":%u,\"locked\":%s,"
            "\"boot_count\":%u,\"buffered\":true,\"ts\":%u}",
            tempBuf, humBuf, rec.battery_pct,
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
        delay(MQTT_FLUSH_SPACING_MS);
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
// Settings — NVS-backed; code values above are first-boot defaults only
// =====================================================================
void settingsLoad() {
    prefs.getString("s_wu", www_username, sizeof(www_username));
    prefs.getString("s_wp", www_password, sizeof(www_password));
    prefs.getString("s_tk", api_token,    sizeof(api_token));
    prefs.getString("s_ms", mqtt_server,  sizeof(mqtt_server));
    prefs.getString("s_mu", mqtt_user,    sizeof(mqtt_user));
    prefs.getString("s_mk", mqtt_pass,    sizeof(mqtt_pass));
    mqtt_port = prefs.getInt ("s_mp", mqtt_port);
    masterKey = prefs.getUInt("s_key", masterKey);
}

// =====================================================================
// OTA rollback — DIY 3-strike counter (stock Arduino core lacks the
// bootloader rollback option). "fw_try" is set to 1 right after an OTA
// write; each unvalidated boot increments it; 3 failures boot the
// previous bank. Cleared after 60s healthy uptime or a completed cycle.
// =====================================================================
void fwRollbackCheck() {
    uint8_t tries = prefs.getUChar("fw_try", 0);
    if (tries == 0) return;
    if (tries >= 3) {
        const esp_partition_t* other = esp_ota_get_next_update_partition(NULL);
        prefs.putUChar("fw_try", 0);
        if (other && esp_ota_set_boot_partition(other) == ESP_OK) {
            Serial.println("[OTA] New firmware failed 3 boots — ROLLING BACK");
            ESP.restart();
        }
    } else {
        prefs.putUChar("fw_try", tries + 1);
        fwPending = true;
        Serial.printf("[OTA] Firmware validation boot %u/3\n", tries);
    }
}

void fwMarkValid() {
    if (!fwPending) return;
    fwPending = false;
    prefs.putUChar("fw_try", 0);
    addLog("Firmware validated");
    Serial.println("[OTA] Firmware marked valid");
}

// =====================================================================
// USB detection — this board uses a CH343 USB-UART bridge. When USB
// power is present the bridge drives UART0 RX (GPIO44) idle-HIGH; with
// our pulldown enabled it reads LOW when the cable is out. While USB
// powers the board the battery divider reads the 5V rail, so battery
// sampling must pause.
// =====================================================================
#define UART_RX_GPIO GPIO_NUM_44

bool usbHostPresent() {
    static unsigned long lastChkMs = 0;
    static bool          present   = false;
    if (millis() - lastChkMs >= 1000 || lastChkMs == 0) {
        lastChkMs = millis();
        present   = (gpio_get_level(UART_RX_GPIO) == 1);
    }
    return present;
}

// =====================================================================
// WiFi Manager implementation
// =====================================================================
void setupServerRoutes();          // fwd
void wifiConnectStart(bool apFallback);  // fwd

void wifiSaveNets() {
    prefs.putBytes("w_nets", wifiNets, sizeof(wifiNets));
    prefs.putInt("w_n", wifiNetCount);
}

void wifiLoadNets() {
    if (prefs.getBytesLength("w_nets") == sizeof(wifiNets)) {
        prefs.getBytes("w_nets", wifiNets, sizeof(wifiNets));
        wifiNetCount = constrain(prefs.getInt("w_n", 0), 0, WIFI_MAX_NETS);
    }
    if (wifiNetCount == 0) {  // first boot — seed from firmware defaults
        strlcpy(wifiNets[0].ssid, ssid, sizeof(wifiNets[0].ssid));
        strlcpy(wifiNets[0].pass, password, sizeof(wifiNets[0].pass));
        wifiNetCount = 1;
        wifiSaveNets();
    }
    Serial.printf("[WiFi] %d stored network(s)\n", wifiNetCount);
}

bool wifiAddNet(const char* s, const char* p) {
    if (!s || !*s) return false;
    for (int i = 0; i < wifiNetCount; i++)
        if (!strcmp(wifiNets[i].ssid, s)) {
            strlcpy(wifiNets[i].pass, p, sizeof(wifiNets[i].pass));
            wifiSaveNets(); return true;
        }
    if (wifiNetCount >= WIFI_MAX_NETS) return false;
    strlcpy(wifiNets[wifiNetCount].ssid, s, sizeof(wifiNets[0].ssid));
    strlcpy(wifiNets[wifiNetCount].pass, p, sizeof(wifiNets[0].pass));
    wifiNetCount++;
    wifiSaveNets();
    return true;
}

bool wifiDelNet(const char* s) {
    for (int i = 0; i < wifiNetCount; i++)
        if (!strcmp(wifiNets[i].ssid, s)) {
            for (int j = i; j < wifiNetCount - 1; j++) wifiNets[j] = wifiNets[j + 1];
            memset(&wifiNets[--wifiNetCount], 0, sizeof(WifiNet));
            if (wifiLastGood == i) wifiLastGood = -1;
            wifiSaveNets(); return true;
        }
    return false;
}

void ensureServerStarted() {
    if (serverStarted) return;
    setupServerRoutes();
    server.begin();
    serverStarted = true;
    Serial.println("[WEB] Server started");
}

void startAP() {
    if (apActive) return;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID);      // open network for setup
    apActive      = true;
    apStartMs     = millis();
    apPrevClients = 0;
    if (!mdnsStarted) { MDNS.begin("sbs"); mdnsStarted = true; }
    ensureServerStarted();
    // Captive portal — any DNS lookup resolves to us, phones pop the page
    if (!dnsServer) {
        dnsServer = new DNSServer();
        dnsServer->start(53, "*", WiFi.softAPIP());
    }
    addLog("Setup AP started (SBS)");
    Serial.printf("[WiFi] AP \"%s\" up — http://192.168.4.1 / http://sbs.local\n", AP_SSID);
}

void stopAP() {
    if (!apActive) return;
    if (dnsServer) { dnsServer->stop(); delete dnsServer; dnsServer = nullptr; }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apActive = false;
    addLog("Setup AP stopped");
    Serial.println("[WiFi] AP stopped");
    // Resume searching for known networks right away
    if (wifiMode != WM_CONNECTED && wifiMode != WM_CONNECTING)
        wifiConnectStart(false);
}

void wifiStartAttempt() {
    wifiCurIdx = (wifiLastGood >= 0 && wifiLastGood < wifiNetCount)
                 ? (wifiLastGood + wifiTryIdx) % wifiNetCount
                 : wifiTryIdx;
    Serial.printf("[WiFi] Trying \"%s\" (%d/%d)\n",
                  wifiNets[wifiCurIdx].ssid, wifiTryIdx + 1, wifiNetCount);
    WiFi.begin(wifiNets[wifiCurIdx].ssid, wifiNets[wifiCurIdx].pass);
    wifiTryStart = millis();
}

void wifiConnectStart(bool apFallback) {
    if (wifiNetCount == 0) { if (apFallback) startAP(); return; }
    WiFi.mode(apActive ? WIFI_AP_STA : WIFI_STA);
    wifiApFallback = apFallback;
    wifiTryIdx     = 0;
    wifiMode       = WM_CONNECTING;
    wifiStartAttempt();
}

// Blocking variant — used only on timer wake (headless report cycle)
bool wifiConnectBlocking(uint32_t totalTimeoutMs) {
    WiFi.mode(WIFI_STA);
    unsigned long start = millis();
    for (int i = 0; i < wifiNetCount && millis() - start < totalTimeoutMs; i++) {
        int idx = (wifiLastGood >= 0 && wifiLastGood < wifiNetCount)
                  ? (wifiLastGood + i) % wifiNetCount : i;
        WiFi.begin(wifiNets[idx].ssid, wifiNets[idx].pass);
        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TRY_MS &&
               millis() - start < totalTimeoutMs) delay(100);
        if (WiFi.status() == WL_CONNECTED) { wifiLastGood = idx; return true; }
        WiFi.disconnect(false, false);
    }
    return false;
}

void onWifiConnected() {
    wifiMode     = WM_CONNECTED;
    wifiLastGood = wifiCurIdx;
    Serial.printf("[WiFi] Connected: \"%s\"  IP %s\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    // Restart mDNS on every (re)connect, not just the first — the ESP32
    // responder can go silent after a WiFi drop/reconnect and never
    // recover on its own, which is why sbs.local sometimes stops
    // resolving even though the device is reachable by IP.
    if (mdnsStarted) MDNS.end();
    MDNS.begin("sbs");
    mdnsStarted = true;
    configTime(2 * 3600, 3600, "pool.ntp.org", "time.cloudflare.com");
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setBufferSize(512);
    mqttClient.setSocketTimeout(2);
    lastMqttTryMs = 0;               // connect on next netTick
    ensureServerStarted();
    if (apActive) stopAP();          // configured net reached — AP no longer needed
    char buf[52];
    snprintf(buf, sizeof(buf), "WiFi: %s", WiFi.localIP().toString().c_str());
    addLog(buf);
}

// Non-blocking connection FSM — called every loop
void wifiTick() {
    if (wifiMode == WM_CONNECTING) {
        if (WiFi.status() == WL_CONNECTED) { onWifiConnected(); return; }
        if (millis() - wifiTryStart >= WIFI_TRY_MS) {
            WiFi.disconnect(false, false);
            wifiTryIdx++;
            if (wifiTryIdx >= wifiNetCount) {
                Serial.println("[WiFi] All stored networks failed");
                wifiMode     = WM_LOST;
                lastWifiKick = millis();
                if (wifiApFallback && !apActive) startAP();
            } else {
                wifiStartAttempt();
            }
        }
    } else if (wifiMode == WM_CONNECTED) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WiFi] Link lost");
            wifiMode     = WM_LOST;
            lastWifiKick = millis();
        }
    } else if (wifiMode == WM_LOST) {
        // While the AP is up, apScanTick() reconnects via a gentle scan
        // instead of blind connect rounds that would kick AP clients off
        if (!apActive && millis() - lastWifiKick >= WIFI_RETRY_MS)
            wifiConnectStart(false);
    }
}

// Proactive scan while in AP mode: if a stored network reappears,
// connect to it even when a client is attached to the AP. Scan-first is
// gentler on AP clients than repeated WiFi.begin() rounds.
void apScanTick() {
    if (!apActive || wifiMode == WM_CONNECTING || wifiMode == WM_CONNECTED) return;
    if (millis() < uiScanUntil) return;   // don't fight a user-initiated scan
    if (!apScanPending) {
        if (millis() - lastApScanMs >= 25000) {
            lastApScanMs = millis();
            WiFi.scanNetworks(true);      // async — AP stays up (AP_STA)
            apScanPending = true;
        }
        return;
    }
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) return;
    apScanPending = false;
    if (n <= 0) { WiFi.scanDelete(); return; }
    int best = -1, bestRssi = -999;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < wifiNetCount; j++)
            if (WiFi.SSID(i) == wifiNets[j].ssid && WiFi.RSSI(i) > bestRssi) {
                best = j; bestRssi = WiFi.RSSI(i);
            }
    WiFi.scanDelete();
    if (best >= 0) {
        Serial.printf("[WiFi] Stored net \"%s\" visible (%d dBm) — connecting\n",
                      wifiNets[best].ssid, bestRssi);
        wifiLastGood = best;              // round starts with this network
        wifiConnectStart(false);
    }
}

// NTP + MQTT — non-blocking, only when STA link is up
void netTick() {
    if (wifiMode != WM_CONNECTED) return;

    if (ntpEpoch == 0) {
        static unsigned long lastNtpChk = 0;
        if (millis() - lastNtpChk >= 500) {
            lastNtpChk = millis();
            time_t now = 0; time(&now);
            if (now > 1000000L) {
                ntpEpoch  = (uint32_t)now;
                ntpMillis = millis();
                Serial.printf("[NTP] Synced: %lu\n", (unsigned long)now);
                plogFixTimestamps();
            }
        }
    }

    if (!mqttClient.connected()) {
        if (lastMqttTryMs == 0 || millis() - lastMqttTryMs >= MQTT_RETRY_MS) {
            lastMqttTryMs = millis();
            if (mqttClient.connect(mqtt_clientid, mqtt_user, mqtt_pass)) {
                Serial.println("[MQTT] Connected");
                if (!mqttEverConnected) { mqttPublishDiscovery(); mqttEverConnected = true; }
                mqttFlushBuffer();
                mqttPublishState(cachedTemp, cachedHum,
                                 batPctAvg >= 0 ? batPctAvg : getBatteryPct(),
                                 currentState != LOCK_OPEN);
            } else {
                Serial.printf("[MQTT] Connect failed rc=%d\n", mqttClient.state());
            }
        }
    } else {
        mqttClient.loop();
    }
}

// AP lifetime — close after 2 min with no client; keep alive while in use.
// When a client disconnects, immediately retry the stored networks (the
// user has likely just configured one).
void apTick() {
    if (!apActive) return;
    if (dnsServer) dnsServer->processNextRequest();
    int clients = WiFi.softAPgetStationNum();
    if (clients > 0) {
        apStartMs     = millis();   // client present — hold AP open
        activityTimer = millis();   // and block deep sleep
    } else if (apPrevClients > 0) {
        Serial.println("[WiFi] AP client left — retrying stored networks");
        if (wifiMode != WM_CONNECTED && wifiMode != WM_CONNECTING)
            wifiConnectStart(false);
    } else if (millis() - apStartMs >= AP_TIMEOUT_MS) {
        Serial.println("[WiFi] AP timeout — no client joined");
        stopAP();
    }
    apPrevClients = clients;
}

// =====================================================================
// Servo Control — angular servo on SERVO_PIN (GPIO 8)
// A = 0 deg, B = 180 deg. motorDirSwapped: true -> A(0)=lock, B(180)=unlock
// Reverted from 90 back to 180 — the mechanism needs the full swing to
// fully seat; 90 left it straining against a hard stop indefinitely
// after locking (servo.detach() alone doesn't stop a jammed mechanism).
// =====================================================================
#define SERVO_DEG_A 0
#define SERVO_DEG_B 180

void servoUnlock() {
    int angle = motorDirSwapped ? SERVO_DEG_B : SERVO_DEG_A;
    Serial.printf("[SERVO] Unlock -> %d deg\n", angle);
    servo.attach(SERVO_PIN);
    servo.write(angle);
}
void servoLock() {
    int angle = motorDirSwapped ? SERVO_DEG_A : SERVO_DEG_B;
    Serial.printf("[SERVO] Lock -> %d deg\n", angle);
    servo.attach(SERVO_PIN);
    servo.write(angle);
}

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
:root{--bg:#0a0a12;--s1:#12121e;--s2:#1a1a2c;--bd:#272740;--bd2:#3b3b64;--ac:#6366f1;--ac2:#8b5cf6;--ag:rgba(99,102,241,.3);--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--tx:#e8ecf4;--tm:#6b7490}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,-apple-system,sans-serif;background:radial-gradient(900px 420px at 50% -10%,#16163a 0%,var(--bg) 62%) fixed;color:var(--tx);min-height:100vh;padding:16px 16px 34px;max-width:480px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:12px 0 16px;margin-bottom:6px}
.hdr h1{font-size:1.32rem;font-weight:800;letter-spacing:-.02em;background:linear-gradient(90deg,#fff 30%,#a7aefb);-webkit-background-clip:text;background-clip:text;-webkit-text-fill-color:transparent}
.hdr .sub{color:var(--tm);font-size:.68rem;margin-top:3px}
.chip{display:inline-flex;align-items:center;gap:6px;padding:5px 12px;border-radius:20px;font-size:.7rem;font-weight:600;border:1px solid;font-family:monospace}
.chip.on{background:rgba(16,185,129,.1);color:var(--gr);border-color:rgba(16,185,129,.3)}
.chip.apm{background:rgba(245,158,11,.1);color:var(--yw);border-color:rgba(245,158,11,.3)}
.chip.off{background:rgba(239,68,68,.08);color:var(--rd);border-color:rgba(239,68,68,.25)}
.chip .dot{width:6px;height:6px;border-radius:50%;background:currentColor;animation:pl 2.2s ease-in-out infinite}
@keyframes pl{50%{opacity:.35}}
.hero{background:linear-gradient(165deg,var(--s2),var(--s1) 70%);border:1px solid var(--bd);border-radius:18px;padding:24px 18px 18px;text-align:center;margin-bottom:12px;position:relative;overflow:hidden}
.hero::before{content:'';position:absolute;left:20%;right:20%;top:-60px;height:120px;background:radial-gradient(60% 100% at 50% 0,rgba(99,102,241,.22),transparent);pointer-events:none}
.lock-ico{width:78px;height:78px;margin:0 auto 12px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:2.1rem;transition:all .4s;position:relative}
.lock-ico.lk{background:rgba(239,68,68,.1);border:2px solid rgba(239,68,68,.35);box-shadow:0 0 34px rgba(239,68,68,.14)}
.lock-ico.ul{background:rgba(16,185,129,.1);border:2px solid rgba(16,185,129,.45);box-shadow:0 0 34px rgba(16,185,129,.3);animation:pg 1s ease-in-out 3}
@keyframes pg{50%{box-shadow:0 0 52px rgba(16,185,129,.55)}}
.lock-h2{font-size:1.18rem;font-weight:700;margin-bottom:2px}
.lock-p{color:var(--tm);font-size:.75rem;margin-bottom:16px}
.btn-ul{width:100%;padding:14px;background:linear-gradient(135deg,var(--ac),var(--ac2));border:none;border-radius:12px;color:#fff;font-size:.95rem;font-weight:700;cursor:pointer;transition:all .2s;letter-spacing:.02em;box-shadow:0 6px 22px rgba(99,102,241,.28)}
.btn-ul:hover{transform:translateY(-1px);box-shadow:0 10px 30px rgba(99,102,241,.42)}
.btn-ul:active{transform:translateY(0)}
.btn-ul:disabled{opacity:.45;cursor:not-allowed;transform:none;box-shadow:none}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:4px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:14px;padding:14px;transition:all .25s}
.card:hover{border-color:var(--bd2);transform:translateY(-1px)}
.lbl{color:var(--tm);font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;margin-bottom:8px}
.val{font-size:1.5rem;font-weight:700;line-height:1;font-variant-numeric:tabular-nums}
.sub2{color:var(--tm);font-size:.7rem;margin-top:5px}
.bat-span{grid-column:span 2}
.bat-bar{height:8px;background:var(--bd);border-radius:4px;margin-top:9px;overflow:hidden}
.bat-fill{height:100%;border-radius:4px;transition:width .5s}
.bat-fill.hi{background:linear-gradient(90deg,#0b9e6b,var(--gr))}.bat-fill.md{background:linear-gradient(90deg,#d18708,var(--yw))}.bat-fill.lo{background:var(--rd)}
.wifi{display:flex;align-items:flex-end;gap:3px;height:22px;margin-bottom:4px}
.wifi span{width:5px;background:var(--bd);border-radius:2px;transition:background .3s}
.wifi span:nth-child(1){height:6px}.wifi span:nth-child(2){height:11px}.wifi span:nth-child(3){height:16px}.wifi span:nth-child(4){height:21px}
.wifi.gd span{background:var(--gr)}.wifi.md span:not(:nth-child(4)){background:var(--yw)}.wifi.wk span:first-child{background:var(--rd)}
.nav{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:12px 0 2px}
.nav a{text-align:center;padding:11px 2px 9px;background:var(--s1);border:1px solid var(--bd);border-radius:12px;color:var(--tm);font-size:.6rem;font-weight:600;text-decoration:none;transition:all .2s;display:flex;flex-direction:column;align-items:center;gap:4px}
.nav a .ic{font-size:1.05rem;line-height:1}
.nav a:hover{border-color:var(--ac);color:var(--tx);transform:translateY(-1px)}
.sec{font-size:.65rem;text-transform:uppercase;letter-spacing:.12em;color:var(--tm);margin:18px 0 7px;display:flex;align-items:center;gap:8px}
.sec::after{content:'';flex:1;height:1px;background:var(--bd)}
.lst{background:var(--s1);border:1px solid var(--bd);border-radius:14px;overflow:hidden}
.li{display:flex;align-items:center;justify-content:space-between;padding:11px 14px;border-bottom:1px solid var(--bd);font-size:.83rem}
.li:last-child{border-bottom:none}
.li .id{color:var(--tm);font-family:monospace;font-size:.72rem}
.btn-del{background:rgba(239,68,68,.1);border:1px solid rgba(239,68,68,.28);color:var(--rd);padding:4px 10px;border-radius:6px;font-size:.72rem;cursor:pointer;transition:all .2s}
.btn-del:hover{background:rgba(239,68,68,.2)}
.log-it{padding:9px 14px;border-bottom:1px solid var(--bd);font-size:.78rem;display:flex;gap:10px;align-items:baseline}
.log-it:last-child{border-bottom:none}
.log-t{color:var(--tm);font-family:monospace;font-size:.66rem;white-space:nowrap}
.log-it.ok .log-m{color:var(--gr)}.log-it.er .log-m{color:var(--rd)}.log-it.in .log-m{color:var(--tx)}
.spin{display:inline-block;width:13px;height:13px;border:2px solid var(--bd);border-top-color:var(--ac);border-radius:50%;animation:sp .8s linear infinite}
@keyframes sp{to{transform:rotate(360deg)}}
.empty{padding:22px;text-align:center;color:var(--tm);font-size:.82rem}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%) translateY(80px);background:var(--s2);border:1px solid var(--bd);padding:11px 22px;border-radius:10px;font-size:.82rem;transition:transform .3s;z-index:99;white-space:nowrap;box-shadow:0 10px 30px rgba(0,0,0,.45)}
.toast.sh{transform:translateX(-50%) translateY(0)}
.toast.ok{border-color:var(--gr);color:var(--gr)}.toast.er{border-color:var(--rd);color:var(--rd)}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>SmartSafe Pro</h1><div class="sub" id="uptime">Loading...</div></div>
  <div class="chip off" id="netChip"><span class="dot"></span><span id="netTxt">...</span></div>
</div>
<div class="hero">
  <div class="lock-ico lk" id="lIco">&#128274;</div>
  <div class="lock-h2" id="lStat">Locked</div>
  <div class="lock-p" id="lSub">State: Idle</div>
  <button class="btn-ul" id="ulBtn" onclick="doUnlock()">&#128275; Unlock Door</button>
</div>
<div class="g2">
  <div class="card"><div class="lbl">&#127777; Temperature</div><div class="val" id="tVal">--</div><div class="sub2">&#176;C</div></div>
  <div class="card">
    <div class="lbl">&#128246; WiFi</div>
    <div class="wifi gd" id="wBars"><span></span><span></span><span></span><span></span></div>
    <div class="sub2" id="wRssi">-- dBm</div>
  </div>
  <div class="card bat-span">
    <div class="lbl">&#128267; Battery <span id="usbBadge" style="display:none;color:var(--yw)">&#9889; USB</span></div>
    <div class="val" id="bVal">--%</div>
    <div class="bat-bar"><div class="bat-fill hi" id="bBar" style="width:0%"></div></div>
    <div id="bDbg" style="font-family:monospace;font-size:.62rem;color:var(--yw);margin-top:5px;line-height:1.5">raw: --<br>-- V</div>
  </div>
</div>
<div class="nav">
  <a href="/monitor"><span class="ic">&#128270;</span>Monitor</a>
  <a href="/graph"><span class="ic">&#128200;</span>History</a>
  <a href="/calib"><span class="ic">&#128295;</span>Calib</a>
  <a href="/wifi"><span class="ic">&#128246;</span>WiFi</a>
  <a href="/update"><span class="ic">&#11014;</span>OTA</a>
  <a href="/settings"><span class="ic">&#9881;</span>Settings</a>
</div>
<div class="sec">Authorized Cards</div>
<div class="lst" id="cList"><div class="empty"><span class="spin"></span></div></div>
<div class="sec">Activity Log</div>
<div class="lst" id="lList"><div class="empty"><span class="spin"></span></div></div>
<div class="toast" id="toast"></div>
<script>
let cd=false;
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
    document.getElementById('usbBadge').style.display=d.usb?'inline':'none';
    if(d.bat_raw!==undefined)document.getElementById('bDbg').innerHTML='raw: '+d.bat_raw+'<br>'+parseFloat(d.bat_v).toFixed(3)+' V';
    document.getElementById('wRssi').textContent=d.rssi+' dBm';
    const wb=document.getElementById('wBars');
    wb.className='wifi '+(d.rssi>-60?'gd':d.rssi>-75?'md':'wk');
    const s=d.uptime,h=Math.floor(s/3600),m=Math.floor((s%3600)/60);
    document.getElementById('uptime').textContent='Uptime: '+(h?h+'h ':'')+m+'m · v'+(d.fw||'?');
    const nc=document.getElementById('netChip'),nt=document.getElementById('netTxt');
    if(d.net==='sta'){nc.className='chip on';nt.textContent=d.ip;}
    else if(d.net==='ap'){nc.className='chip apm';nt.textContent='AP · SBS';}
    else{nc.className='chip off';nt.textContent='Offline';}
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
    const r=await fetch('/open');
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
.stat{font-size:.72rem;color:var(--tm)}.stat b{color:var(--tx);font-weight:600;margin-left:3px}
.legend{display:flex;gap:12px;flex-wrap:wrap;margin-top:10px}
.leg{font-size:.65rem;display:flex;align-items:center;gap:4px;color:var(--tm)}
.leg-dot{width:8px;height:8px;border-radius:50%;display:inline-block}
.ev-row{display:flex;gap:10px;align-items:center;padding:5px 0;border-bottom:1px solid var(--bd);font-size:.75rem}
.ev-row:last-child{border-bottom:none}
.ev-time{color:var(--tm);min-width:80px;font-size:.68rem}
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
  <div class="legend">
    <div class="leg"><span class="leg-dot" style="background:#10b981"></span>Unlock</div>
    <div class="leg"><span class="leg-dot" style="background:#f87171"></span>Lock</div>
    <div class="leg"><span class="leg-dot" style="background:#fb923c"></span>Denied</div>
    <div class="leg"><span class="leg-dot" style="background:#818cf8"></span>Boot</div>
  </div>
</div>
<div class="card">
  <div class="chart-lbl">Recent Events</div>
  <div id="evList"><div class="empty">Loading...</div></div>
</div>
<script>
const PL=34,PR=8,PT=12,PB=22,IH=96,W=460,IW=W-PL-PR;
const EV_COL={1:'#10b981',2:'#f87171',3:'#fb923c',4:'#818cf8'};
const EV_LBL={1:'Unlocked',2:'Locked',3:'Denied',4:'Boot'};
function chart(pts,evs,yLo,yHi,stroke,unit){
  if(!pts||pts.length<2)return'<div class="empty">Not enough data</div>';
  const n=pts.length;
  const t0=pts[0].ts,t1=pts[n-1].ts,tSpan=t1-t0||1;
  const xByTs=ts=>(PL+(ts-t0)/tSpan*IW).toFixed(1);
  const xS=i=>xByTs(pts[i].ts);
  const yS=v=>(PT+IH-(v-yLo)/(yHi-yLo||1)*IH).toFixed(1);
  const steps=4;let g='';
  for(let i=0;i<=steps;i++){
    const v=yLo+(yHi-yLo)/steps*i,y=(PT+IH-(v-yLo)/(yHi-yLo||1)*IH).toFixed(1);
    g+=`<line x1="${PL}" y1="${y}" x2="${W-PR}" y2="${y}" stroke="#2a2a45" stroke-width="1"/>`;
    g+=`<text x="${PL-4}" y="${(+y+3.5).toFixed(1)}" text-anchor="end" font-size="9" fill="#64748b">${unit==='%'?v.toFixed(0):v.toFixed(1)}</text>`;
  }
  const tStep=Math.max(1,Math.floor(n/5));
  for(let i=0;i<n;i+=tStep){
    const x=xS(i),p=pts[i];
    const lbl=p.ts>86400?(d=>d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0'))(new Date(p.ts*1000)):(Math.floor(p.ts/3600)+'h');
    g+=`<text x="${x}" y="${PT+IH+14}" text-anchor="middle" font-size="9" fill="#64748b">${lbl}</text>`;
  }
  let area=`M${xS(0)},${yS(pts[0].v)}`,line=`M${xS(0)},${yS(pts[0].v)}`;
  for(let i=1;i<n;i++){area+=` L${xS(i)},${yS(pts[i].v)}`;line+=` L${xS(i)},${yS(pts[i].v)}`;}
  area+=` L${xS(n-1)},${(PT+IH).toFixed(1)} L${xS(0)},${(PT+IH).toFixed(1)} Z`;
  const lx=xS(n-1),ly=yS(pts[n-1].v);
  let marks='';
  for(const ev of evs){
    if(!EV_COL[ev.evt])continue;
    const r=(ev.ts-t0)/tSpan;
    if(r<0||r>1)continue;
    const ex=(PL+r*IW).toFixed(1),c=EV_COL[ev.evt];
    marks+=`<line x1="${ex}" y1="${PT}" x2="${ex}" y2="${PT+IH}" stroke="${c}" stroke-width="1" stroke-dasharray="3,3" opacity="0.5"/>`;
    marks+=`<circle cx="${ex}" cy="${PT}" r="4" fill="${c}" opacity="0.85"/>`;
  }
  return`<svg viewBox="0 0 ${W} ${PT+IH+PB}" xmlns="http://www.w3.org/2000/svg">${g}${marks}<path d="${area}" fill="${stroke}" fill-opacity=".07"/><path d="${line}" fill="none" stroke="${stroke}" stroke-width="1.5" stroke-linejoin="round"/><circle cx="${lx}" cy="${ly}" r="3" fill="${stroke}"/></svg>`;
}
function fmtTime(ts){
  if(ts>86400){const d=new Date(ts*1000);return d.toLocaleDateString('en-GB',{month:'short',day:'numeric'})+' '+d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0');}
  return Math.floor(ts/60)+'min';
}
async function load(){
  try{
    const d=await(await fetch('/api/graph')).json();
    if(!d||!d.length){
      ['tChart','bChart'].forEach(id=>document.getElementById(id).innerHTML='<div class="empty">No data yet &#8212; recorded every 10 min</div>');
      document.getElementById('subEl').textContent='0 points';
      document.getElementById('evList').innerHTML='<div class="empty">No events recorded</div>';
      return;
    }
    const evs=d.filter(p=>p.evt>0);
    document.getElementById('subEl').textContent=d.length+' point'+(d.length===1?'':'s')+' · up to 24h (NVS)';
    const tp=d.filter(p=>p.temp!==null).map(p=>({ts:p.ts,v:p.temp}));
    if(tp.length>=2){
      const vs=tp.map(p=>p.v),mn=Math.min(...vs),mx=Math.max(...vs),pad=Math.max(.5,(mx-mn)*.15);
      document.getElementById('tChart').innerHTML=chart(tp,evs,mn-pad,mx+pad,'#06b6d4','C');
      const last=tp[tp.length-1];
      document.getElementById('tStats').innerHTML=`<div class="stat">Now<b>${last.v.toFixed(1)}&#176;C</b></div><div class="stat">Min<b>${mn.toFixed(1)}&#176;C</b></div><div class="stat">Max<b>${mx.toFixed(1)}&#176;C</b></div>`;
    }else document.getElementById('tChart').innerHTML='<div class="empty">Not enough data</div>';
    const bp=d.map(p=>({ts:p.ts,v:p.bat}));
    const bvs=bp.map(p=>p.v),bmn=Math.max(0,Math.min(...bvs)-5),bmx=Math.min(100,Math.max(...bvs)+5);
    document.getElementById('bChart').innerHTML=chart(bp,evs,bmn,bmx,'#10b981','%');
    const blast=bp[bp.length-1];
    document.getElementById('bStats').innerHTML=`<div class="stat">Now<b>${blast.v}%</b></div><div class="stat">Min<b>${Math.min(...bvs)}%</b></div><div class="stat">Max<b>${Math.max(...bvs)}%</b></div>`;
    const evDiv=document.getElementById('evList');
    if(evs.length){
      evDiv.innerHTML=[...evs].reverse().slice(0,30).map(ev=>`<div class="ev-row"><span class="ev-time">${fmtTime(ev.ts)}</span><span style="color:${EV_COL[ev.evt]||'var(--tx)'}">${EV_LBL[ev.evt]||'Event'}</span></div>`).join('');
    }else{
      evDiv.innerHTML='<div class="empty" style="padding:12px">No events in this window</div>';
    }
  }catch(e){
    ['tChart','bChart'].forEach(id=>document.getElementById(id).innerHTML='<div class="empty">Failed to load</div>');
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
<title>Calibration &#8212; SmartSafe</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--bd:#2a2a45;--ac:#6366f1;--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:440px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:16px}
.hdr h1{font-size:1.15rem;font-weight:700}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.sec{font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin:16px 0 6px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:16px;margin-bottom:8px}
.lbl{color:var(--tm);font-size:.65rem;margin-bottom:8px}
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
.angle{font-size:1.4rem;font-weight:700;font-family:monospace;color:var(--yw);margin-bottom:4px}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#9881; Calibration</h1></div>
  <a href="/" class="back">&#8592; Dashboard</a>
</div>

<div class="step-num">Step 1</div>
<div class="sec">Test positions</div>
<div class="card">
  <div class="lbl">Move servo to each endpoint to see which physically opens/closes the lock</div>
  <div class="row">
    <button class="btn ba" onclick="api('cmd=posA')">&#9654; Position A &mdash; 0&deg;</button>
    <button class="btn bb" onclick="api('cmd=posB')">&#9654; Position B &mdash; 180&deg;</button>
    <button class="btn bstop" onclick="api('cmd=stop')" title="Detach">&#9632;</button>
  </div>
</div>

<div class="step-num">Step 2</div>
<div class="sec">Set open position</div>
<div class="card">
  <div class="lbl">Select which position opens the lock</div>
  <div class="row">
    <button class="btn ba" onclick="setDir('A')">A (0&deg;) = Open &#10003;</button>
    <button class="btn bb" onclick="setDir('B')">B (180&deg;) = Open &#10003;</button>
  </div>
  <div id="dirChip"><span class="chip chip-none">Not set yet</span></div>
</div>

<div class="step-num">Step 3</div>
<div class="sec">Test open / close</div>
<div class="card">
  <div class="lbl">Verify both positions work correctly</div>
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

<div class="sec">Battery Voltage Calibration</div>
<div class="card">
  <div class="lbl">Measure the actual battery voltage with a multimeter, then enter it here to correct for divider/ADC tolerance</div>
  <div class="row" style="align-items:center">
    <span>Device reading</span>
    <span class="mono" id="batLiveV" style="color:var(--yw)">--</span>
  </div>
  <div class="row">
    <input type="number" step="0.01" id="batMeasured" placeholder="Multimeter volts, e.g. 4.18" style="flex:1;padding:11px;background:var(--bg);border:1px solid var(--bd);border-radius:8px;color:var(--tx);font-size:.9rem">
  </div>
  <div class="row">
    <button class="btn ba" onclick="batCalibrate()">&#9989; Calibrate</button>
    <button class="btn bstop" style="flex:0 0 90px" onclick="batReset()">Reset</button>
  </div>
  <div class="lbl" style="margin-top:8px">Current correction factor: <span class="mono" id="batFactor">--</span></div>
  <div id="batMsg"></div>
</div>

<script>
let unlockDir=null;
async function api(p){try{const r=await fetch('/api/calib?'+p);return await r.json();}catch(e){return{ok:false};}}
async function batRefresh(){
  const r=await api('cmd=batread');
  if(r&&r.ok){
    document.getElementById('batLiveV').textContent=r.cal_v.toFixed(3)+'V';
    document.getElementById('batFactor').textContent=r.factor.toFixed(4);
  }
}
async function batCalibrate(){
  const v=parseFloat(document.getElementById('batMeasured').value);
  const el=document.getElementById('batMsg');
  if(!v||v<2||v>5){el.innerHTML='<div class="msg msg-er">Enter a valid voltage (2&#8211;5V)</div>';return;}
  const r=await api('cmd=batcal&v='+v);
  el.innerHTML=r&&r.ok
    ?'<div class="msg msg-ok">&#10003; Calibrated &#8212; factor '+r.factor.toFixed(4)+'</div>'
    :'<div class="msg msg-er">&#10007; Calibration failed</div>';
  batRefresh();
}
async function batReset(){
  const r=await api('cmd=batcalreset');
  const el=document.getElementById('batMsg');
  el.innerHTML=r&&r.ok?'<div class="msg msg-ok">&#10003; Reset to factory (1.0000)</div>':'<div class="msg msg-er">&#10007; Reset failed</div>';
  batRefresh();
}
batRefresh();
setInterval(batRefresh,3000);
function setDir(d){
  unlockDir=d;
  document.getElementById('dirChip').innerHTML='<span class="chip chip-ok">Position '+d+(d==='A'?' (0&deg;)':' (180&deg;)')+' = Open &#10003;</span>';
  document.getElementById('btnOpen').disabled=false;
  document.getElementById('btnClose').disabled=false;
  document.getElementById('btnSave').disabled=false;
}
function testAction(a){
  const isA=(a==='open')?(unlockDir==='A'):(unlockDir==='B');
  api('cmd='+(isA?'posA':'posB'));
}
async function save(){
  const r=await api('cmd=save&dir='+unlockDir);
  const el=document.getElementById('saveMsg');
  el.innerHTML=r&&r.ok
    ?'<div class="msg msg-ok">&#10003; Saved! Position '+unlockDir+(unlockDir==='A'?' (0&deg;)':' (180&deg;)')+' opens the lock</div>'
    :'<div class="msg msg-er">&#10007; Save failed</div>';
}
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// HTML — WiFi Settings
// =====================================================================
const char WIFI_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>WiFi &#8212; SmartSafe</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--bd:#2a2a45;--ac:#6366f1;--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:440px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:16px}
.hdr h1{font-size:1.15rem;font-weight:700}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.sec{font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin:16px 0 6px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:14px;margin-bottom:8px}
.row{display:flex;justify-content:space-between;align-items:center;padding:9px 0;border-bottom:1px solid rgba(42,42,69,.6);font-size:.83rem}
.row:last-child{border-bottom:none}
.mono{font-family:monospace;color:var(--tm);font-size:.72rem}
.rssi{font-size:.68rem;color:var(--tm);min-width:56px;text-align:right}
.btn-del{background:rgba(239,68,68,.1);border:1px solid rgba(239,68,68,.28);color:var(--rd);padding:4px 10px;border-radius:6px;font-size:.72rem;cursor:pointer}
.btn-add{background:rgba(16,185,129,.1);border:1px solid rgba(16,185,129,.28);color:var(--gr);padding:4px 10px;border-radius:6px;font-size:.72rem;cursor:pointer}
.bscan{width:100%;padding:13px;background:linear-gradient(135deg,#6366f1,#8b5cf6);border:none;border-radius:12px;color:#fff;font-size:.92rem;font-weight:700;cursor:pointer}
.bscan:disabled{opacity:.4;cursor:not-allowed}
.empty{padding:18px;text-align:center;color:var(--tm);font-size:.8rem}
.chip{display:inline-flex;padding:3px 10px;border-radius:20px;font-size:.7rem;font-weight:600}
.chip-ok{background:rgba(16,185,129,.12);color:var(--gr);border:1px solid rgba(16,185,129,.3)}
.chip-off{background:rgba(100,116,139,.12);color:var(--tm);border:1px solid rgba(100,116,139,.3)}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%) translateY(80px);background:#1a1a2e;border:1px solid var(--bd);padding:11px 22px;border-radius:8px;font-size:.82rem;transition:transform .3s;z-index:99;white-space:nowrap}
.toast.sh{transform:translateX(-50%) translateY(0)}
.toast.ok{border-color:var(--gr);color:var(--gr)}.toast.er{border-color:var(--rd);color:var(--rd)}
.spin{display:inline-block;width:13px;height:13px;border:2px solid var(--bd);border-top-color:var(--ac);border-radius:50%;animation:sp .8s linear infinite;vertical-align:-2px}
@keyframes sp{to{transform:rotate(360deg)}}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#128246; WiFi Settings</h1></div>
  <a href="/" class="back">&#8592; Dashboard</a>
</div>
<div class="sec">Connection</div>
<div class="card" id="stCard"><div class="empty"><span class="spin"></span></div></div>
<div class="sec">Saved Networks (max 5)</div>
<div class="card" id="savedCard"><div class="empty">Loading...</div></div>
<div class="sec">Add a Network</div>
<button class="bscan" id="scanBtn" onclick="doScan()">&#128269; Scan for Networks</button>
<div class="card" id="scanCard" style="margin-top:8px"><div class="empty">Press Scan to search</div></div>
<div class="toast" id="toast"></div>
<script>
function toast(m,t){const el=document.getElementById('toast');el.textContent=m;el.className='toast sh '+(t||'');setTimeout(()=>el.classList.remove('sh'),3200);}
async function api(p){const r=await fetch(p);if(!r.ok)throw new Error(r.status);return r.json();}
async function loadStatus(){
  try{
    const d=await api('/api/wifi/status');
    let h='';
    if(d.mode==='sta'){
      h=`<div class="row"><span>Status</span><span class="chip chip-ok">Connected</span></div>
         <div class="row"><span>Network</span><span class="mono">${d.ssid}</span></div>
         <div class="row"><span>IP</span><span class="mono">${d.ip}</span></div>
         <div class="row"><span>Signal</span><span class="mono">${d.rssi} dBm</span></div>`;
    }else if(d.mode==='connecting'){
      h=`<div class="row"><span>Status</span><span class="chip chip-off"><span class="spin"></span>&nbsp;Connecting...</span></div>`;
    }else{
      h=`<div class="row"><span>Status</span><span class="chip chip-off">Not connected</span></div>`;
    }
    if(d.ap)h+=`<div class="row"><span>Setup AP</span><span class="chip chip-ok">SBS active · ${d.ap_clients} client(s)</span></div>`;
    document.getElementById('stCard').innerHTML=h;
    const sc=document.getElementById('savedCard');
    if(!d.nets.length)sc.innerHTML='<div class="empty">No saved networks</div>';
    else sc.innerHTML=d.nets.map(n=>`<div class="row"><span>${n}${d.mode==='sta'&&n===d.ssid?' <span class="chip chip-ok">&#10003;</span>':''}</span><button class="btn-del" onclick="delNet('${n.replace(/'/g,"\\'")}')">Remove</button></div>`).join('');
  }catch(e){}
}
async function delNet(s){
  if(!confirm('Remove network "'+s+'"?'))return;
  try{const d=await api('/api/wifi/del?ssid='+encodeURIComponent(s));
    if(d.ok){toast('Network removed','ok');loadStatus();}else toast('Failed','er');
  }catch(e){toast('Error','er');}
}
let scanning=false;
async function doScan(){
  if(scanning)return;scanning=true;
  const btn=document.getElementById('scanBtn'),card=document.getElementById('scanCard');
  btn.disabled=true;card.innerHTML='<div class="empty"><span class="spin"></span> Scanning...</div>';
  try{
    await api('/api/wifi/scan?start=1');
    for(let i=0;i<20;i++){
      await new Promise(r=>setTimeout(r,700));
      const d=await api('/api/wifi/scan');
      if(d.running)continue;
      if(!d.nets||!d.nets.length){card.innerHTML='<div class="empty">No networks found</div>';break;}
      card.innerHTML=d.nets.map(n=>`<div class="row"><span>${n.sec?'&#128274; ':''}${n.ssid}</span><span style="display:flex;gap:8px;align-items:center"><span class="rssi">${n.rssi} dBm</span><button class="btn-add" onclick="addNet('${n.ssid.replace(/'/g,"\\'")}',${n.sec})">Add</button></span></div>`).join('');
      break;
    }
  }catch(e){card.innerHTML='<div class="empty">Scan failed</div>';}
  btn.disabled=false;scanning=false;
}
async function addNet(s,sec){
  let p='';
  if(sec){p=prompt('Password for "'+s+'":');if(p===null)return;}
  try{
    const d=await api('/api/wifi/add?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p));
    if(d.ok){toast('Saved — connecting...','ok');setTimeout(loadStatus,1500);}
    else toast(d.err||'Failed (5 max?)','er');
  }catch(e){toast('Error','er');}
}
loadStatus();setInterval(loadStatus,4000);
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// HTML — OTA Update
// =====================================================================
const char OTA_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>OTA Update &#8212; SmartSafe</title>
<style>
:root{--bg:#0a0a14;--s1:#13131f;--bd:#2a2a45;--ac:#6366f1;--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--tx:#e2e8f0;--tm:#64748b}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:480px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:24px}
.hdr h1{font-size:1.15rem;font-weight:700}.hdr .sub{color:var(--tm);font-size:.68rem;margin-top:2px}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:20px;margin-bottom:16px}
.card h2{font-size:.85rem;font-weight:600;margin-bottom:14px}
.card p{font-size:.76rem;color:var(--tm);margin-bottom:14px;line-height:1.5}
.drop{border:2px dashed var(--bd);border-radius:10px;padding:30px;text-align:center;cursor:pointer;transition:border .2s}
.drop:hover,.drop.over{border-color:var(--ac)}
.drop input{display:none}
.drop-lbl{font-size:.8rem;color:var(--tm)}
.drop-name{font-size:.8rem;color:var(--ac);margin-top:6px;font-weight:600}
.btn{width:100%;margin-top:14px;background:var(--ac);color:#fff;border:none;border-radius:8px;padding:11px;font-size:.85rem;font-weight:600;cursor:pointer}
.btn:disabled{opacity:.4;cursor:default}
.prog{margin-top:14px;display:none}
.prog-bar{height:6px;background:var(--bd);border-radius:3px;overflow:hidden}
.prog-fill{height:100%;width:0;background:var(--ac);transition:width .3s}
.prog-txt{font-size:.72rem;color:var(--tm);margin-top:6px;text-align:center}
.msg{margin-top:12px;font-size:.8rem;text-align:center;padding:8px;border-radius:8px;display:none}
.msg.ok{background:rgba(16,185,129,.12);color:var(--gr);border:1px solid rgba(16,185,129,.3)}
.msg.er{background:rgba(239,68,68,.1);color:var(--rd);border:1px solid rgba(239,68,68,.3)}
.warn{background:rgba(245,158,11,.1);border:1px solid rgba(245,158,11,.3);border-radius:8px;padding:10px 12px;font-size:.74rem;color:var(--yw);margin-bottom:16px}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#11014; OTA Update</h1><div class="sub" id="fwEl">Current firmware: ...</div></div>
  <a class="back" href="/">&#8592; Back</a>
</div>
<div class="warn">&#9888; The device reboots after a successful upload. Make sure the door is locked first.</div>
<div class="card">
  <h2>Upload Firmware (.bin)</h2>
  <p>PlatformIO output: <code style="color:var(--ac)">.pio/build/esp32s3/firmware.bin</code></p>
  <div class="drop" id="drop" onclick="document.getElementById('file').click()" ondragover="event.preventDefault();this.classList.add('over')" ondragleave="this.classList.remove('over')" ondrop="onDrop(event)">
    <input type="file" id="file" accept=".bin" onchange="onFile(this.files[0])">
    <div class="drop-lbl">&#128194; Click or drop .bin file here</div>
    <div class="drop-name" id="fname"></div>
  </div>
  <button class="btn" id="btn" disabled onclick="doUpload()">Upload &amp; Update</button>
  <div class="prog" id="prog"><div class="prog-bar"><div class="prog-fill" id="fill"></div></div><div class="prog-txt" id="ptxt">0%</div></div>
  <div class="msg" id="msg"></div>
</div>
<script>
let file=null;
fetch('/api/status').then(r=>r.json()).then(d=>{document.getElementById('fwEl').textContent='Current firmware: v'+(d.fw||'?');}).catch(()=>{});
function onDrop(e){e.preventDefault();document.getElementById('drop').classList.remove('over');onFile(e.dataTransfer.files[0]);}
function onFile(f){if(!f||!f.name.endsWith('.bin'))return;file=f;document.getElementById('fname').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';document.getElementById('btn').disabled=false;}
function showMsg(txt,ok){const m=document.getElementById('msg');m.textContent=txt;m.className='msg '+(ok?'ok':'er');m.style.display='block';}
function doUpload(){
  if(!file)return;
  const btn=document.getElementById('btn');btn.disabled=true;
  const fd=new FormData();fd.append('firmware',file,file.name);
  const xhr=new XMLHttpRequest();
  xhr.open('POST','/update');
  xhr.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded/e.total*100);document.getElementById('fill').style.width=p+'%';document.getElementById('ptxt').textContent=p+'%';}};
  document.getElementById('prog').style.display='block';
  xhr.onload=()=>{if(xhr.status===200&&xhr.responseText.startsWith('OK')){showMsg('Success! Rebooting... reconnect in ~5s.',true);}else{showMsg('Failed: '+xhr.responseText,false);btn.disabled=false;}};
  xhr.onerror=()=>{showMsg('Connection error',false);btn.disabled=false;};
  xhr.send(fd);
}
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// HTML — Settings
// =====================================================================
const char SETTINGS_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Settings &#8212; SmartSafe</title>
<style>
:root{--bg:#0a0a12;--s1:#12121e;--bd:#272740;--ac:#6366f1;--ac2:#8b5cf6;--gr:#10b981;--rd:#ef4444;--yw:#f59e0b;--tx:#e8ecf4;--tm:#6b7490}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh;padding:16px;max-width:440px;margin:0 auto}
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 0 20px;border-bottom:1px solid var(--bd);margin-bottom:16px}
.hdr h1{font-size:1.15rem;font-weight:700}
.back{color:var(--ac);font-size:.78rem;text-decoration:none;padding:5px 11px;border:1px solid var(--bd);border-radius:8px}
.sec{font-size:.62rem;text-transform:uppercase;letter-spacing:.1em;color:var(--tm);margin:16px 0 6px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:12px;padding:14px;margin-bottom:8px}
label{display:block;color:var(--tm);font-size:.65rem;text-transform:uppercase;letter-spacing:.08em;margin:10px 0 5px}
label:first-child{margin-top:0}
input{width:100%;padding:10px 12px;background:var(--bg);border:1px solid var(--bd);border-radius:8px;color:var(--tx);font-size:.9rem;font-family:monospace}
input:focus{outline:none;border-color:var(--ac)}
input::placeholder{color:#3a3a5c}
.note{font-size:.68rem;color:var(--tm);margin-top:8px;line-height:1.5}
.bsave{width:100%;padding:15px;background:linear-gradient(135deg,var(--ac),var(--ac2));border:none;border-radius:12px;color:#fff;font-size:1rem;font-weight:700;cursor:pointer;margin-top:10px}
.bsave:disabled{opacity:.4}
.msg{margin-top:10px;padding:10px 12px;border-radius:8px;font-size:.82rem;text-align:center;display:none}
.msg.ok{background:rgba(16,185,129,.1);color:var(--gr);border:1px solid rgba(16,185,129,.2);display:block}
.msg.er{background:rgba(239,68,68,.1);color:var(--rd);border:1px solid rgba(239,68,68,.2);display:block}
.warn{background:rgba(245,158,11,.08);border:1px solid rgba(245,158,11,.25);border-radius:8px;padding:9px 12px;font-size:.7rem;color:var(--yw);margin-bottom:12px;line-height:1.5}
</style>
</head>
<body>
<div class="hdr">
  <div><h1>&#9881; Settings</h1></div>
  <a href="/" class="back">&#8592; Dashboard</a>
</div>
<div class="warn">&#9888; Values are stored on the device (NVS) and survive updates. Leave a password field empty to keep the current one.</div>
<div class="sec">Web Access</div>
<div class="card">
  <label>Username</label><input id="wu" maxlength="23" autocomplete="off">
  <label>Password</label><input id="wp" type="password" maxlength="31" placeholder="(unchanged)" autocomplete="new-password">
  <div class="note">Changing these will make the browser ask you to log in again.</div>
</div>
<div class="sec">API Token</div>
<div class="card">
  <label>Token (for /open?t=... automation)</label><input id="tk" maxlength="32" autocomplete="off">
</div>
<div class="sec">MQTT — Home Assistant</div>
<div class="card">
  <label>Server</label><input id="ms" maxlength="39" autocomplete="off">
  <label>Port</label><input id="mp" type="number" min="1" max="65535">
  <label>Username</label><input id="mu" maxlength="23" autocomplete="off">
  <label>Password</label><input id="mk" type="password" maxlength="31" placeholder="(unchanged)" autocomplete="new-password">
</div>
<div class="sec">RFID</div>
<div class="card">
  <label>Master key (card # that toggles edit mode)</label><input id="key" type="number" min="1">
</div>
<button class="bsave" id="btnSave" onclick="save()">&#128190; Save Settings</button>
<div class="msg" id="msg"></div>
<script>
async function load(){
  try{
    const d=await(await fetch('/api/settings')).json();
    document.getElementById('wu').value=d.wu;
    document.getElementById('tk').value=d.tk;
    document.getElementById('ms').value=d.ms;
    document.getElementById('mp').value=d.mp;
    document.getElementById('mu').value=d.mu;
    document.getElementById('key').value=d.key;
  }catch(e){}
}
async function save(){
  const btn=document.getElementById('btnSave');btn.disabled=true;
  const f=id=>encodeURIComponent(document.getElementById(id).value.trim());
  const q=`wu=${f('wu')}&wp=${f('wp')}&tk=${f('tk')}&ms=${f('ms')}&mp=${f('mp')}&mu=${f('mu')}&mk=${f('mk')}&key=${f('key')}`;
  const m=document.getElementById('msg');
  try{
    const r=await(await fetch('/api/settings/save?'+q)).json();
    m.className='msg '+(r.ok?'ok':'er');
    m.textContent=r.ok?'✓ Saved! Settings applied.':'✗ Save failed';
    document.getElementById('wp').value='';document.getElementById('mk').value='';
  }catch(e){m.className='msg er';m.textContent='Connection error';}
  btn.disabled=false;
}
load();
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
// WS2812B LED Ring
// =====================================================================
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum LedEffect { LED_OFF, LED_PAUSE, LED_RAINBOW, LED_BLUE_FADE,
                 LED_GREEN_BLINK, LED_RED_BLINK,
                 LED_GREEN_FADE,  LED_RED_FADE };

LedEffect     ledEffect        = LED_OFF;
LedEffect     ledReturnEffect  = LED_OFF;
LedEffect     ledPendingEffect = LED_OFF;
LedEffect     ledPendingReturn = LED_OFF;
uint32_t      ledPauseDuration = 0;
unsigned long ledEffectStart   = 0;
uint16_t      rainbowHue       = 0;

// pauseMs: blank delay before starting effect.
// When leaving RAINBOW, at least 200ms is enforced to settle DIN.
void ledSetEffect(LedEffect effect, LedEffect returnTo = LED_OFF, uint32_t pauseMs = 0) {
    uint32_t pause = pauseMs;
    if (ledEffect == LED_RAINBOW && effect != LED_OFF && effect != LED_RAINBOW)
        pause = (pause > 200) ? pause : 200;
    if (pause > 0) {
        ledPendingEffect = effect;
        ledPendingReturn = returnTo;
        ledPauseDuration = pause;
        ledEffect        = LED_PAUSE;
        ledEffectStart   = millis();
    } else {
        ledEffect       = effect;
        ledReturnEffect = returnTo;
        ledEffectStart  = millis();
    }
}

void ledUpdate() {
    unsigned long t = millis() - ledEffectStart;

    switch (ledEffect) {
        case LED_OFF:
            ring.clear(); ring.show(); break;

        case LED_PAUSE:
            ring.clear(); ring.show();
            if (t >= ledPauseDuration) {
                ledEffect       = ledPendingEffect;
                ledReturnEffect = ledPendingReturn;
                ledEffectStart  = millis();
            }
            break;

        case LED_RAINBOW:
            rainbowHue += 256;
            for (int i = 0; i < LED_COUNT; i++) {
                uint16_t hue = rainbowHue + (uint16_t)(i * 65536L / LED_COUNT);
                ring.setPixelColor(i, ring.gamma32(ring.ColorHSV(hue, 255, 200)));
            }
            ring.show(); break;

        case LED_BLUE_FADE: {
            // 1-second cycle: 500ms up + 500ms down, continuous
            const uint32_t period = 1000, half = 500;
            uint8_t br = (t % period < half)
                         ? (uint8_t)((t % period) * 255 / half)
                         : (uint8_t)((period - t % period) * 255 / half);
            uint32_t c = ring.Color(0, 0, br);
            for (int i = 0; i < LED_COUNT; i++) ring.setPixelColor(i, c);
            ring.show(); break;
        }

        case LED_GREEN_BLINK:
        case LED_RED_BLINK: {
            // 3 × 1s (500ms ON + 500ms OFF) = 3s total
            const uint32_t cycle = 1000, on_ms = 500, reps = 3;
            if (t >= cycle * reps) { ledSetEffect(ledReturnEffect); break; }
            uint32_t tc = t % cycle;
            uint32_t c  = (ledEffect == LED_GREEN_BLINK)
                          ? ring.Color(0, 180, 0) : ring.Color(180, 0, 0);
            if (tc < on_ms) { for (int i=0;i<LED_COUNT;i++) ring.setPixelColor(i,c); ring.show(); }
            else            { ring.clear(); ring.show(); }
            break;
        }

        case LED_GREEN_FADE:
        case LED_RED_FADE: {
            // 3 × 1s (500ms fade-in + 500ms fade-out) = 3s total
            const uint32_t cycle = 1000, half = 500, reps = 3;
            if (t >= cycle * reps) { ledSetEffect(ledReturnEffect); break; }
            uint32_t tc = t % cycle;
            uint8_t  br = (tc < half) ? (uint8_t)(tc * 255 / half)
                                      : (uint8_t)((cycle - tc) * 255 / half);
            uint32_t c = (ledEffect == LED_GREEN_FADE)
                         ? ring.Color(0, br, 0) : ring.Color(br, 0, 0);
            for (int i = 0; i < LED_COUNT; i++) ring.setPixelColor(i, c);
            ring.show(); break;
        }
    }
}

// =====================================================================
// FSM
// =====================================================================
void setSystemState(SystemState newState) {
    SystemState prevState = currentState;
    currentState  = newState;
    stateTimer    = millis();
    activityTimer = millis();

    if (newState == IDLE) {
        lockHoldStart = 0;
        digitalWrite(BOOST_12V_EN_PIN, LOW);
        if (prevState == LOCK_OPEN) {
            plogAdd(cachedTemp, (uint8_t)constrain(batPctAvg >= 0 ? batPctAvg : getBatteryPct(), 0, 100), 2);
            servoLock();
            servoDetachAt = millis() + SERVO_DETACH_MS;
        }
        editMode = false;
        ledSetEffect(LED_OFF);
        Serial.println("[FSM] IDLE — locking");
    } else if (newState == READER_ACTIVE) {
        digitalWrite(BOOST_12V_EN_PIN, HIGH);
        boosterStartTime = millis();
        noInterrupts(); v_rfid.bits = 0; v_rfid.code = 0; interrupts();
        ledSetEffect(LED_RAINBOW);
        addLog("RFID reader activated");
        Serial.println("[FSM] READER_ACTIVE");
    } else if (newState == LOCK_OPEN) {
        plogAdd(cachedTemp, (uint8_t)constrain(batPctAvg >= 0 ? batPctAvg : getBatteryPct(), 0, 100), 1);
        servoUnlock();
        // Detach once it reaches the open position — no need to keep
        // driving the servo for the full LOCK_HOLD_MS hold.
        servoDetachAt = millis() + SERVO_DETACH_MS;
        lockHoldStart = millis();
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
        // Token optional (basic-auth already guards); if present it must match
        if (req->hasParam("t") && req->getParam("t")->value() != api_token) {
            addLog("Invalid token attempt");
            req->send(403, "text/plain", "Invalid Token");
            return;
        }
        if (millis() - lastUnlockTime > LOCK_COOLDOWN_MS) {
            // Same LED behavior as an RFID open — ledSetEffect() already
            // enforces a settle pause when leaving LED_RAINBOW, so this is
            // safe to call even while the reader is active.
            ledSetEffect(LED_GREEN_FADE, LED_OFF, 500);
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
        String json; json.reserve(256);
        json += "{\"state\":\"";   json += st;
        json += "\",\"temp\":";    json += isnan(cachedTemp) ? "-99" : String(cachedTemp, 1);
        json += ",\"hum\":";       json += isnan(cachedHum)  ? "-1"  : String(cachedHum,  1);
        json += ",\"battery\":";   json += dispPct;
        json += ",\"bat_raw\":";   json += bat.raw;
        json += ",\"bat_v\":";     json += String(dispVolt, 3);
        json += ",\"rssi\":";      json += rssi;
        json += ",\"uptime\":";    json += millis() / 1000;
        json += ",\"buffered\":";  json += rtcCount;
        json += ",\"usb\":";       json += usbHostPresent() ? "true" : "false";
        json += ",\"net\":\"";
        json += apActive ? "ap" : (wifiMode == WM_CONNECTED ? "sta" : "off");
        json += "\",\"ip\":\"";
        json += apActive ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
        json += "\",\"fw\":\"" FW_VERSION "\"}";
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
        String json; json.reserve(plogCount * 48 + 8);
        json += "[";
        for (int i = 0; i < plogCount; i++) {
            int idx = plogGetIdx(i);
            PLogRecord& r = plogBuf[idx];
            if (i > 0) json += ",";
            json += "{\"ts\":";    json += r.ts;
            json += ",\"temp\":";
            if (r.temp10 == -9990) json += "null";
            else json += String(r.temp10 / 10.0f, 1);
            json += ",\"bat\":";   json += r.bat;
            json += ",\"evt\":";   json += r.evt;
            json += "}";
        }
        json += "]";
        req->send(200, "application/json", json);
    });

    server.on("/monitor", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        req->send_P(200, "text/html", MONITOR_PAGE);
    });

    // ── WiFi management ──────────────────────────────────────────────
    server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        req->send_P(200, "text/html", WIFI_PAGE);
    });

    server.on("/api/wifi/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        String json; json.reserve(384);
        json += "{\"mode\":\"";
        json += (wifiMode == WM_CONNECTED)  ? "sta"
              : (wifiMode == WM_CONNECTING) ? "connecting" : "off";
        json += "\",\"ssid\":\"";  json += WiFi.SSID();
        json += "\",\"ip\":\"";    json += WiFi.localIP().toString();
        json += "\",\"rssi\":";    json += WiFi.RSSI();
        json += ",\"ap\":";        json += apActive ? "true" : "false";
        json += ",\"ap_clients\":"; json += apActive ? WiFi.softAPgetStationNum() : 0;
        json += ",\"nets\":[";
        for (int i = 0; i < wifiNetCount; i++) {
            if (i) json += ",";
            json += "\""; json += wifiNets[i].ssid; json += "\"";
        }
        json += "]}";
        req->send(200, "application/json", json);
    });

    server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        if (req->hasParam("start")) {
            uiScanUntil   = millis() + 15000;  // hold off the AP auto-scan
            apScanPending = false;
            WiFi.scanDelete();
            WiFi.scanNetworks(true);   // async
            req->send(200, "application/json", "{\"started\":true}");
            return;
        }
        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) {
            req->send(200, "application/json", "{\"running\":true}");
            return;
        }
        String json; json.reserve(n > 0 ? n * 64 : 32);
        json += "{\"running\":false,\"nets\":[";
        for (int i = 0; i < n; i++) {
            if (i) json += ",";
            json += "{\"ssid\":\"";  json += WiFi.SSID(i);
            json += "\",\"rssi\":";  json += WiFi.RSSI(i);
            json += ",\"sec\":";     json += (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) ? "true" : "false";
            json += "}";
        }
        json += "]}";
        WiFi.scanDelete();
        req->send(200, "application/json", json);
    });

    server.on("/api/wifi/add", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        String s = req->hasParam("ssid") ? req->getParam("ssid")->value() : "";
        String p = req->hasParam("pass") ? req->getParam("pass")->value() : "";
        if (!s.length()) { req->send(400, "application/json", "{\"ok\":false,\"err\":\"no ssid\"}"); return; }
        if (!wifiAddNet(s.c_str(), p.c_str())) {
            req->send(200, "application/json", "{\"ok\":false,\"err\":\"list full (5)\"}");
            return;
        }
        char buf[52]; snprintf(buf, sizeof(buf), "WiFi net saved: %s", s.c_str());
        addLog(buf);
        // Not connected (or in setup AP) — try the new network right away
        if (wifiMode != WM_CONNECTED) wifiConnectStart(false);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/api/wifi/del", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        String s = req->hasParam("ssid") ? req->getParam("ssid")->value() : "";
        bool ok = wifiDelNet(s.c_str());
        req->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    // ── OTA — web upload (Update.h, no global SSE/EventSource objects)
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        req->send_P(200, "text/html", OTA_PAGE);
    });
    server.on("/update", HTTP_POST,
        [](AsyncWebServerRequest *req) {
            bool ok = !Update.hasError();
            AsyncWebServerResponse *resp = req->beginResponse(200, "text/plain",
                ok ? "OK - rebooting" : "FAILED");
            resp->addHeader("Connection", "close");
            req->send(resp);
            if (ok) { delay(300); ESP.restart(); }
            else otaBusy = false;
        },
        [](AsyncWebServerRequest *req, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            if (!req->authenticate(www_username, www_password)) { req->send(401); return; }
            if (!index) {
                otaBusy = true;
                servo.detach();               // never move the lock mid-update
                Serial.printf("[OTA] Uploading: %s\n", filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) Update.printError(Serial);
            }
            if (len && Update.write(data, len) != len) Update.printError(Serial);
            if (final) {
                if (Update.end(true)) {
                    prefs.putUChar("fw_try", 1);   // arm rollback watchdog
                    Serial.printf("[OTA] Done: %u bytes\n", (unsigned)(index + len));
                } else Update.printError(Serial);
            }
            activityTimer = millis();
        }
    );

    server.on("/calib", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        req->send_P(200, "text/html", CALIB_PAGE);
    });

    server.on("/api/calib", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        String cmd = req->hasParam("cmd") ? req->getParam("cmd")->value() : "";

        if (cmd == "posA") {
            servo.attach(SERVO_PIN);
            servo.write(SERVO_DEG_A);
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "posB") {
            servo.attach(SERVO_PIN);
            servo.write(SERVO_DEG_B);
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "stop") {
            servo.detach();
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "save") {
            String dir  = req->hasParam("dir") ? req->getParam("dir")->value() : "A";
            motorDirSwapped = (dir == "B");
            prefs.putBool("m_dir", motorDirSwapped);
            Serial.printf("[CALIB] saved: dir=%s\n", dir.c_str());
            req->send(200, "application/json", "{\"ok\":true}");

        } else if (cmd == "batread") {
            float raw = (analogReadMilliVolts(BAT_ADC) / 1000.0f) * 2.0f; // uncalibrated
            String json = "{\"ok\":true,\"raw_v\":" + String(raw, 3) +
                          ",\"cal_v\":" + String(raw * batCalFactor, 3) +
                          ",\"factor\":" + String(batCalFactor, 4) + "}";
            req->send(200, "application/json", json);

        } else if (cmd == "batcal") {
            if (!req->hasParam("v")) { req->send(400, "application/json", "{\"ok\":false}"); return; }
            float measured = req->getParam("v")->value().toFloat();
            float raw = (analogReadMilliVolts(BAT_ADC) / 1000.0f) * 2.0f; // uncalibrated
            if (measured <= 0.5f || raw <= 0.5f) {
                req->send(400, "application/json", "{\"ok\":false}");
                return;
            }
            batCalFactor = measured / raw;
            prefs.putFloat("bat_cal", batCalFactor);
            Serial.printf("[CALIB] battery: raw=%.3fV measured=%.3fV factor=%.4f\n", raw, measured, batCalFactor);
            String json = "{\"ok\":true,\"factor\":" + String(batCalFactor, 4) + "}";
            req->send(200, "application/json", json);

        } else if (cmd == "batcalreset") {
            batCalFactor = 1.0f;
            prefs.putFloat("bat_cal", batCalFactor);
            Serial.println("[CALIB] battery: factor reset to 1.0");
            req->send(200, "application/json", "{\"ok\":true,\"factor\":1.0}");

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

    // ── Settings ─────────────────────────────────────────────────────
    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        req->send_P(200, "text/html", SETTINGS_PAGE);
    });

    // NOTE: registered before /api/settings — the library prefix-matches,
    // so the shorter route would otherwise swallow /api/settings/save
    server.on("/api/settings/save", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        activityTimer = millis();
        bool mqttChanged = false;
        auto has = [&](const char* p) {
            return req->hasParam(p) && req->getParam(p)->value().length() > 0;
        };
        if (has("wu")) { strlcpy(www_username, req->getParam("wu")->value().c_str(), sizeof(www_username)); prefs.putString("s_wu", www_username); }
        if (has("wp")) { strlcpy(www_password, req->getParam("wp")->value().c_str(), sizeof(www_password)); prefs.putString("s_wp", www_password); }
        if (has("tk")) { strlcpy(api_token,    req->getParam("tk")->value().c_str(), sizeof(api_token));    prefs.putString("s_tk", api_token); }
        if (has("ms")) { strlcpy(mqtt_server,  req->getParam("ms")->value().c_str(), sizeof(mqtt_server));  prefs.putString("s_ms", mqtt_server); mqttChanged = true; }
        if (has("mu")) { strlcpy(mqtt_user,    req->getParam("mu")->value().c_str(), sizeof(mqtt_user));    prefs.putString("s_mu", mqtt_user);   mqttChanged = true; }
        if (has("mk")) { strlcpy(mqtt_pass,    req->getParam("mk")->value().c_str(), sizeof(mqtt_pass));    prefs.putString("s_mk", mqtt_pass);   mqttChanged = true; }
        if (has("mp")) {
            int p = constrain(req->getParam("mp")->value().toInt(), 1, 65535);
            if (p != mqtt_port) { mqtt_port = p; prefs.putInt("s_mp", p); mqttChanged = true; }
        }
        if (has("key")) {
            uint32_t k = (uint32_t)req->getParam("key")->value().toInt();
            if (k > 0) { masterKey = k; prefs.putUInt("s_key", k); }
        }
        if (mqttChanged) {
            mqttClient.disconnect();
            mqttClient.setServer(mqtt_server, mqtt_port);
            mqttEverConnected = false;   // republish discovery on reconnect
            lastMqttTryMs     = 0;
        }
        addLog("Settings updated");
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->authenticate(www_username, www_password)) return req->requestAuthentication();
        String json; json.reserve(256);
        json += "{\"wu\":\"";  json += www_username;
        json += "\",\"tk\":\""; json += api_token;
        json += "\",\"ms\":\""; json += mqtt_server;
        json += "\",\"mp\":";   json += mqtt_port;
        json += ",\"mu\":\"";   json += mqtt_user;
        json += "\",\"key\":";  json += masterKey;
        json += "}";
        req->send(200, "application/json", json);
    });

    // Captive portal — while the setup AP is active, unknown URLs
    // (phone connectivity probes) are redirected to the WiFi page
    server.onNotFound([](AsyncWebServerRequest *req) {
        if (apActive) {
            AsyncWebServerResponse* r = req->beginResponse(302, "text/plain", "");
            r->addHeader("Location", "http://192.168.4.1/wifi");
            req->send(r);
        } else {
            req->send(404, "text/plain", "Not found");
        }
    });
}

// =====================================================================
// Setup
// =====================================================================
void setup() {
    Serial.begin(115200);
    // USB detection: drop the RX pull-up, add pulldown — line then reads
    // HIGH only while the CH343 bridge is USB-powered (see usbHostPresent)
    gpio_pullup_dis(UART_RX_GPIO);
    gpio_pulldown_en(UART_RX_GPIO);
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
        // Load prefs (incl. battery calibration) BEFORE sampling — otherwise
        // this periodic report ignores the saved calibration factor.
        prefs.begin("safe-app", false);
        settingsLoad();
        fwRollbackCheck();
        batCalFactor = prefs.getFloat("bat_cal", 1.0f);

        ds18.begin();
        ds18.requestTemperatures();
        delay(750);
        float t = ds18.getTempCByIndex(0);
        bool tempValid = (t > -100.0f) && (t >= TEMP_MIN_C && t <= TEMP_MAX_C);
        if (!tempValid) {
            // Read failed or implausible — one retry before giving up
            // (occasional OneWire glitch right after deep-sleep wake).
            delay(50);
            ds18.requestTemperatures();
            delay(750);
            t = ds18.getTempCByIndex(0);
            tempValid = (t > -100.0f) && (t >= TEMP_MIN_C && t <= TEMP_MAX_C);
        }
        if (!tempValid) t = NAN;

        // Median-of-3 + plausibility check, same as the awake sampling
        // pipeline — a single raw ADC sample here was producing spurious
        // spikes (e.g. jumping to 100%) on this unattended report path.
        float vbat = takeBatReading();
        if (vbat < BAT_MIN_V || vbat > BAT_MAX_V) {
            delay(50);
            vbat = takeBatReading();
        }
        int bat = constrain((int)((vbat - 3.0f) / 1.2f * 100.0f), 0, 100);
        Serial.printf("[TIMER] T=%.1f BAT=%d\n", isnan(t) ? -99.0f : t, bat);

        wifiLoadNets();
        wifiConnectBlocking(WIFI_CONNECT_TIMEOUT_MS);

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

        fwMarkValid();   // completed report cycle = healthy firmware
        Serial.println("[SYS] Timer wake done — returning to deep sleep");
        ring.clear(); ring.show();
        rtc_gpio_init(SERVO_PIN_GPIO);
        rtc_gpio_set_direction(SERVO_PIN_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(SERVO_PIN_GPIO, 0);
        rtc_gpio_hold_en(SERVO_PIN_GPIO);
        esp_sleep_enable_ext0_wakeup(TOUCH_GPIO, 1);
        esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_US);
        esp_deep_sleep_start();
    }

    // ── Touch Wake / Boot: normal operation ─────────────────────────
    ring.begin();
    ring.setBrightness(60);
    ring.show();

    pinMode(TOUCH_PIN,        INPUT_PULLDOWN);
    pinMode(BOOST_12V_EN_PIN, OUTPUT);
    pinMode(D0_PIN,           INPUT_PULLUP);
    pinMode(D1_PIN,           INPUT_PULLUP);
    digitalWrite(BOOST_12V_EN_PIN, LOW);

    pinMode(SERVO_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(D0_PIN), rfid_isr_d0, FALLING);
    attachInterrupt(digitalPinToInterrupt(D1_PIN), rfid_isr_d1, FALLING);

    prefs.begin("safe-app", false);
    settingsLoad();
    fwRollbackCheck();
    motorDirSwapped = prefs.getBool("m_dir", true);
    batCalFactor    = prefs.getFloat("bat_cal", 1.0f);
    wifiLoadNets();

    // Release RTC hold on servo pin BEFORE driving it
    if (wakeReason == WAKE_TOUCH) {
        rtc_gpio_hold_dis(SERVO_PIN_GPIO);
        rtc_gpio_deinit(SERVO_PIN_GPIO);
    }

    // Ensure door is locked on every startup/wake
    servoLock();
    servoDetachAt = millis() + SERVO_DETACH_MS;

    // ── Reader is live HERE — everything below is non-blocking ──────
    setSystemState(READER_ACTIVE);
    if (wakeReason == WAKE_TOUCH) {
        addLog("Touch wake — reader activated");
        Serial.printf("[SYS] Reader active %lums after boot\n", millis());
    } else {
        addLog("System started — reader active");
    }

    plogLoad();
    plogAdd(NAN, (uint8_t)constrain(getBatteryPct(), 0, 100), 4); // boot event

    // DS18B20 — async: first reading arrives in loop() ~1s from now
    ds18.begin();
    ds18.setResolution(12);
    ds18.setWaitForConversion(false);
    lastDHTRead = millis() - 4800;   // schedule first request in ~200ms

    // WiFi — non-blocking connect; AP fallback only on touch wake
    wifiConnectStart(wakeReason == WAKE_TOUCH);

    activityTimer = millis();
    Serial.printf("[SYS] Setup done in %lums\n", millis());
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
    // LED ring update — non-blocking, max 50fps
    static unsigned long lastLedMs = 0;
    if (millis() - lastLedMs >= 20) { lastLedMs = millis(); ledUpdate(); }

    // Detach servo after it has had time to reach position
    if (servoDetachAt > 0 && millis() >= servoDetachAt) {
        servo.detach();
        servoDetachAt = 0;
        Serial.println("[SERVO] Detached");
    }

    // Networking — connect FSM, NTP/MQTT, AP lifetime. All non-blocking.
    wifiTick();
    netTick();
    apTick();
    apScanTick();

    // New firmware survives 60s → mark valid (cancels OTA rollback)
    if (fwPending && millis() > 60000) fwMarkValid();

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
        if (t <= -100.0f) {
            Serial.println("[DS18B20] Read FAILED");
        } else if (t < TEMP_MIN_C || t > TEMP_MAX_C) {
            Serial.printf("[DS18B20] %.2f°C outside plausible range, rejected\n", t);
        } else {
            cachedTemp = t;
            Serial.printf("[DS18B20] %.2f°C\n", t);
        }
    }

    // Battery sampling every 10s with per-sample outlier + plausibility check.
    // USB power (PC or a wall charger — electrically indistinguishable,
    // both just supply VBUS) is NOT used to gate sampling: a wall charger
    // legitimately shows a real, valid — if elevated — voltage while
    // charging, and hiding it entirely made the feature worse than useless.
    // Instead, implausible single-cell-LiPo values are rejected directly.
    if (currentState != READER_ACTIVE &&
        millis() - lastBatSampleMs >= BAT_SAMPLE_MS) {
        lastBatSampleMs = millis();
        float v = takeBatReading();
        bool valid = (v >= BAT_MIN_V && v <= BAT_MAX_V);
        if (!valid) {
            Serial.printf("[BAT] %.3fV outside plausible range, resampling\n", v);
            v = takeBatReading();
            valid = (v >= BAT_MIN_V && v <= BAT_MAX_V);
        }
        if (valid && !isnan(batVAvg) && fabsf(v - batVAvg) > BAT_OUTLIER_V) {
            Serial.printf("[BAT] %.3fV deviates from avg %.3fV, resampling\n", v, batVAvg);
            v = takeBatReading();
            if (fabsf(v - batVAvg) > BAT_OUTLIER_V) {
                Serial.printf("[BAT] Resample %.3fV also deviant, skipping\n", v);
                valid = false;
            }
        }
        if (!valid) Serial.printf("[BAT] Sample rejected: %.3fV\n", v);
        if (valid) {
            batSamples[batSampleIdx++] = v;
            Serial.printf("[BAT] Sample[%d] %.3fV\n", batSampleIdx - 1, v);
            if (batSampleIdx >= BAT_SAMPLES) {
                batSampleIdx = 0;
                batVAvg   = filteredBatAverage(batSamples, BAT_SAMPLES);
                batPctAvg = constrain((int)((batVAvg - 3.0f) / 1.2f * 100.0f), 0, 100);
                Serial.printf("[BAT] Avg: %.3fV  %d%%\n", batVAvg, batPctAvg);
            }
        }
    }

    // MQTT periodic (every 10 min, only when averaged data is ready)
    if (batPctAvg >= 0 && mqttClient.connected() &&
        millis() - lastMqttMs >= MQTT_PERIODIC_MS) {
        lastMqttMs = millis();
        mqttPublishState(cachedTemp, cachedHum, batPctAvg, currentState != LOCK_OPEN);
    }

    // Persistent log sensor (every 10 min, independent of MQTT)
    if (batPctAvg >= 0 && millis() - lastPlogMs >= MQTT_PERIODIC_MS) {
        lastPlogMs = millis();
        plogAdd(cachedTemp, (uint8_t)batPctAvg, 0);
    }

    // Touch button
    static bool lastTouch = LOW;
    bool touch = digitalRead(TOUCH_PIN);
    if (touch == HIGH && lastTouch == LOW) {
        activityTimer = millis();
        if (currentState == IDLE)               setSystemState(READER_ACTIVE);
        else if (currentState == READER_ACTIVE) stateTimer = millis();
        // Touch is the trigger for the setup-AP fallback when offline
        if (wifiMode == WM_OFF || wifiMode == WM_LOST) wifiConnectStart(true);
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
                    ledSetEffect(editMode ? LED_BLUE_FADE : LED_RAINBOW);
                    addLog(editMode ? "Edit mode ON" : "Edit mode OFF");
                } else if (editMode) {
                    if (!cardExists(finalCode)) {
                        cardAdd(finalCode);
                        ledSetEffect(LED_GREEN_BLINK, LED_BLUE_FADE);
                        char buf[48]; snprintf(buf, sizeof(buf), "Card #%u registered", finalCode);
                        addLog(buf);
                    } else {
                        cardRemove(finalCode);
                        ledSetEffect(LED_RED_BLINK, LED_BLUE_FADE);
                        char buf[48]; snprintf(buf, sizeof(buf), "Card #%u removed", finalCode);
                        addLog(buf);
                    }
                    stateTimer = millis(); activityTimer = millis();
                } else {
                    if (cardExists(finalCode)) {
                        ledSetEffect(LED_GREEN_FADE, LED_OFF, 500);
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
                        ledSetEffect(LED_RED_FADE, LED_RAINBOW, 500);
                        char buf[52]; snprintf(buf, sizeof(buf), "Denied: card #%u", finalCode);
                        addLog(buf);
                        plogAdd(cachedTemp, (uint8_t)constrain(batPctAvg >= 0 ? batPctAvg : getBatteryPct(), 0, 100), 3);
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

    // Auto-lock 10s after motor finishes opening
    if (currentState == LOCK_OPEN && lockHoldStart > 0 && millis() - lockHoldStart > LOCK_HOLD_MS) {
        addLog("Door locked");
        setSystemState(IDLE);
        if (mqttClient.connected())
            mqttPublishState(cachedTemp, cachedHum, getBatteryPct(), true);
    }

    // Idle timeout → deep sleep (never while OTA runs or setup AP is up)
    if (currentState == IDLE && !otaBusy && !apActive &&
        millis() - activityTimer > SLEEP_TIMEOUT_MS) {
        fwMarkValid();   // made it to a clean sleep = healthy firmware
        addLog("Idle timeout — going to sleep");
        Serial.printf("[SYS] No activity for %lus — entering deep sleep\n",
                      (millis() - activityTimer) / 1000);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        servo.detach();
        ring.clear(); ring.show();
        rtc_gpio_init(SERVO_PIN_GPIO);
        rtc_gpio_set_direction(SERVO_PIN_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(SERVO_PIN_GPIO, 0);
        rtc_gpio_hold_en(SERVO_PIN_GPIO);
        esp_sleep_enable_ext0_wakeup(TOUCH_GPIO, 1);
        esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_US);
        esp_deep_sleep_start();
    }

    yield();
}
