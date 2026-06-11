#include <Arduino.h>
#include <WiFi.h>
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
#define LOCK_OPEN_MS        3000
#define LOCK_COOLDOWN_MS    2000
#define READER_TIMEOUT_MS   15000
#define WIEGAND_TIMEOUT_US  200000
#define BOOSTER_SETTLING_MS 200
#define SLEEP_TIMEOUT_MS    60000

// דיווח MQTT אחת לשעה
#define REPORT_INTERVAL_US      (3600ULL * 1000000ULL)
#define WIFI_CONNECT_TIMEOUT_MS 10000

// =====================================================================
// הגדרות רשת
// =====================================================================
const char* ssid         = "Kedmi";
const char* password     = "0504241190";
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
PubSubClient   mqttClient(wifiClient);

float         cachedTemp  = NAN;
float         cachedHum   = NAN;
unsigned long lastDHTRead = 0;

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
int getBatteryPct() {
    int raw = analogRead(BAT_ADC);
    float vbat = (raw / 4095.0f) * 3.3f * 2.0f;
    return constrain((int)((vbat - 3.0f) / 1.2f * 100.0f), 0, 100);
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
        digitalWrite(LOCK_SSR, LOW);
        editMode = false;
        Serial.println("[FSM] IDLE");
    } else if (newState == READER_ACTIVE) {
        digitalWrite(BOOST_12V_EN_PIN, HIGH);
        boosterStartTime = millis();
        noInterrupts(); v_rfid.bits = 0; v_rfid.code = 0; interrupts();
        addLog("קורא RFID הופעל");
        Serial.println("[FSM] READER_ACTIVE");
    } else if (newState == LOCK_OPEN) {
        digitalWrite(LOCK_SSR, HIGH);
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
        int   bat  = getBatteryPct();
        int   rssi = WiFi.RSSI();
        const char* st = currentState == LOCK_OPEN     ? "LOCK_OPEN"
                       : currentState == READER_ACTIVE ? "READER_ACTIVE" : "IDLE";
        String json = "{\"state\":\""; json += st;
        json += "\",\"temp\":";    json += isnan(cachedTemp) ? "-99" : String(cachedTemp, 1);
        json += ",\"hum\":";       json += isnan(cachedHum)  ? "-1"  : String(cachedHum,  1);
        json += ",\"battery\":";   json += bat;
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
        WiFi.begin(ssid, password);
        unsigned long wStart = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - wStart < WIFI_CONNECT_TIMEOUT_MS)
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

        esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_PIN, 1);
        esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_US);
        Serial.println("[SYS] Back to sleep");
        delay(50);
        esp_deep_sleep_start();
        return;
    }

    // ── Touch Wake / Boot: פעולה רגילה ──────────────────────────────
    pinMode(TOUCH_PIN, INPUT_PULLDOWN);
    pinMode(BOOST_12V_EN_PIN, OUTPUT);
    pinMode(LOCK_SSR, OUTPUT);
    pinMode(D0_PIN, INPUT_PULLUP);
    pinMode(D1_PIN, INPUT_PULLUP);
    digitalWrite(BOOST_12V_EN_PIN, LOW);
    digitalWrite(LOCK_SSR, LOW);

    attachInterrupt(digitalPinToInterrupt(D0_PIN), rfid_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(D1_PIN), rfid_isr, FALLING);

    prefs.begin("safe-app", false);
    dht.begin();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(500);

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

    esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_PIN, 1);
    esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_US);
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
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

    // Deep sleep
    if (millis() - activityTimer > SLEEP_TIMEOUT_MS) {
        addLog("כניסה למצב שינה");
        Serial.println("[SYS] Deep Sleep");
        rtcAddRecord(cachedTemp, cachedHum, getBatteryPct(), 0, 0);
        if (mqttClient.connected()) mqttClient.disconnect();
        WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
        digitalWrite(BOOST_12V_EN_PIN, LOW); digitalWrite(LOCK_SSR, LOW);
        btStop();
        esp_wifi_stop();
        delay(100);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_PIN, 1);
        esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_US);
        esp_deep_sleep_start();
    }

    yield();
}
