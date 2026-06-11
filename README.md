# SmartSafe Pro 🔒

**ESP32-based smart safe controller** with RFID card access, Home Assistant integration, remote web unlock, temperature/humidity monitoring, and deep-sleep power management.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware](#hardware)
- [Wiring Diagram](#wiring-diagram)
- [Software Stack](#software-stack)
- [Configuration](#configuration)
- [Web Dashboard](#web-dashboard)
- [REST API](#rest-api)
- [MQTT / Home Assistant Integration](#mqtt--home-assistant-integration)
- [State Machine](#state-machine)
- [Power Management](#power-management)
- [RTC Data Buffer](#rtc-data-buffer)
- [Building & Flashing](#building--flashing)
- [Security Notes](#security-notes)

---

## Overview

SmartSafe Pro turns an ESP32 into a full-featured electronic safe controller. It reads **Wiegand 26-bit RFID cards**, serves a **Hebrew-language web dashboard** over Wi-Fi, publishes sensor and access events to **Home Assistant via MQTT autodiscovery**, and aggressively conserves battery with **ESP32 deep sleep** — waking on touch or on a 1-hour timer.

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32 DevKit                         │
│                                                             │
│  RFID Reader ─────── Wiegand D0/D1 (GPIO 5 / 13)          │
│  Touch Button ─────── GPIO 32 (EXT0 wake source)           │
│  12 V Boost Enable ── GPIO 14                               │
│  Lock SSR ──────────── GPIO 25                              │
│  DHT11 ─────────────── GPIO 4                               │
│  Battery ADC ────────── GPIO 35 (voltage divider ÷2)       │
│  LED Red / Green ──── GPIO 19 / 18                         │
│                                                             │
│  Web UI  ──── http://smartsafe.local  (HTTP Basic Auth)    │
│  MQTT    ──── Home Assistant autodiscovery on port 1883    │
└─────────────────────────────────────────────────────────────┘
```

---

## Features

| Category | Details |
|---|---|
| **Access control** | Wiegand 26-bit RFID reader; unlimited cards stored in NVS flash |
| **Edit mode** | Present master card to toggle registration/deletion of other cards |
| **Remote unlock** | Authenticated HTTP GET with API token; 2 s cooldown protection |
| **Environmental** | DHT11 temperature & humidity, sampled every 3 s |
| **Battery** | ADC voltage divider on GPIO 35, mapped to 0–100 % |
| **Web dashboard** | Single-page app (Hebrew RTL), live polling every 5 s |
| **MQTT** | Home Assistant MQTT autodiscovery for temperature, humidity, battery, lock state, last event, boot count |
| **Deep sleep** | Sleeps after 60 s idle; wakes on GPIO touch or 1-hour timer |
| **Offline buffering** | Up to 200 sensor records survive deep sleep in RTC RAM and are flushed to MQTT on next connection |

---

## Hardware

| Component | Notes |
|---|---|
| ESP32 DevKit (38-pin or 30-pin) | Any variant with GPIO 32 capable of EXT0 wake |
| Wiegand RFID reader (125 kHz or 13.56 MHz) | DATA0 → GPIO 5, DATA1 → GPIO 13 |
| 12 V electromagnetic lock or strike | Driven via SSR on GPIO 25 |
| Boost converter enable | MOSFET gate on GPIO 14; powers 12 V lock rail |
| DHT11 sensor | GPIO 4, 3.3 V supply |
| Voltage divider (2× equal resistors) | Halves battery voltage into GPIO 35 (max 3.3 V) |
| Touch / wake button | GPIO 32, active HIGH |
| Dual LED (red / green) | GPIO 19 / 18, 220 Ω series resistors |

> **Note:** GPIO 12 is avoided for the second Wiegand line — it affects the boot strapping voltage on some ESP32 modules.

---

## Wiring Diagram

```
Battery (+)──┬──[2 MΩ]──┬──GPIO 35
             │           │
             │         [2 MΩ]
             │           │
             │          GND
             │
             └── Boost converter Vin
                      │
               GPIO 14 → MOSFET gate → Boost EN
                      │
               Boost Vout (12 V) → Lock (+)
                                         │
               GPIO 25 → SSR → Lock (−)

RFID Reader:
  VCC  → 5 V (or 3.3 V depending on reader)
  GND  → GND
  D0   → GPIO 5
  D1   → GPIO 13

DHT11:
  VCC → 3.3 V
  GND → GND
  DAT → GPIO 4  (4.7 kΩ pull-up to 3.3 V)

Touch button:
  One leg → GPIO 32
  Other   → 3.3 V  (INPUT_PULLDOWN configured in firmware)
```

---

## Software Stack

```
PlatformIO / Arduino framework on espressif32
│
├── ESPAsyncWebServer-esphome  ^3.1.0   — async HTTP server
├── DHT sensor library          ^1.4.6   — DHT11/22 driver
├── Adafruit Unified Sensor     ^1.1.14  — sensor abstraction
└── PubSubClient                ^2.8     — MQTT client
```

See [platformio.ini](platformio.ini) for the full build configuration.

---

## Configuration

> **Do not commit credentials to version control.** Move the block below into a `include/secrets.h` file and add it to `.gitignore`.

```cpp
// include/secrets.h  (excluded from git)
#pragma once

// Wi-Fi
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Web dashboard (HTTP Basic Auth)
const char* www_username = "admin";
const char* www_password = "YOUR_WEB_PASSWORD";

// REST API token  (sent as ?t=<token> in /open requests)
const char* api_token = "YOUR_API_TOKEN";

// MQTT broker (Home Assistant IP)
const char* mqtt_server = "192.168.x.x";
const char* mqtt_user   = "YOUR_MQTT_USER";
const char* mqtt_pass   = "YOUR_MQTT_PASSWORD";

// Master RFID card code (uint32) — toggles edit mode
uint32_t masterKey = YOUR_MASTER_CARD_CODE;
```

### Timing constants (`main.cpp`)

| Macro | Default | Purpose |
|---|---|---|
| `LOCK_OPEN_MS` | 3000 ms | How long the lock stays open after auth |
| `LOCK_COOLDOWN_MS` | 2000 ms | Minimum time between consecutive unlocks |
| `READER_TIMEOUT_MS` | 15 000 ms | RFID reader auto-shutoff if no card presented |
| `WIEGAND_TIMEOUT_US` | 200 000 µs | Inter-bit silence that marks end of Wiegand frame |
| `BOOSTER_SETTLING_MS` | 200 ms | Grace period after enabling 12 V rail before reading RFID bits |
| `SLEEP_TIMEOUT_MS` | 60 000 ms | Idle time before entering deep sleep |
| `REPORT_INTERVAL_US` | 3 600 s | Deep-sleep timer wake interval for hourly MQTT reporting |

---

## Web Dashboard

Served at `http://smartsafe.local` (mDNS) or the device's IP address on port 80. Protected by HTTP Basic Authentication.

**Live widgets:**
- Lock state icon (animates on unlock)
- Temperature, humidity, battery percentage with bar
- Wi-Fi RSSI signal strength bars
- System uptime

**Card management:**
- Lists all authorised RFID cards stored in NVS
- One-click delete button per card

**Activity log:**
- Last 15 events, colour-coded (green = access granted, red = denied/error)
- Timestamps formatted as `HH:MM:SS` uptime

**Remote unlock button:**
- Sends `GET /open?t=<token>` with the configured API token
- Blocked during 2-second cooldown with HTTP 429

---

## REST API

All endpoints require HTTP Basic Auth.

| Method | Endpoint | Description | Response |
|---|---|---|---|
| `GET` | `/` | Serve HTML dashboard | `200 text/html` |
| `GET` | `/open?t=<token>` | Unlock the safe | `200 OK`, `403 Invalid Token`, `429 Cooldown` |
| `GET` | `/api/status` | JSON system state | see below |
| `GET` | `/api/cards` | List authorised cards | `{"cards":["12345","67890"]}` |
| `POST` | `/api/cards/delete?id=<code>` | Remove a card | `200 OK`, `404 Not found` |
| `GET` | `/api/log` | Activity log (newest first) | `{"log":[{"time":"00:01:23","msg":"..."}]}` |

### `/api/status` response

```json
{
  "state":    "IDLE",
  "temp":     24.5,
  "hum":      55,
  "battery":  78,
  "rssi":     -62,
  "uptime":   143,
  "buffered": 0
}
```

`state` is one of `IDLE`, `READER_ACTIVE`, `LOCK_OPEN`.

---

## MQTT / Home Assistant Integration

The device publishes MQTT autodiscovery messages on first connect, so entities appear in Home Assistant automatically under the **SmartSafe Pro** device.

### Topics

| Topic | Payload | Retained |
|---|---|---|
| `homeassistant/sensor/smartsafe_pro/temperature/config` | HA discovery JSON | ✓ |
| `homeassistant/sensor/smartsafe_pro/humidity/config` | HA discovery JSON | ✓ |
| `homeassistant/sensor/smartsafe_pro/battery/config` | HA discovery JSON | ✓ |
| `homeassistant/binary_sensor/smartsafe_pro/lock/config` | HA discovery JSON | ✓ |
| `homeassistant/sensor/smartsafe_pro/last_event/config` | HA discovery JSON | ✓ |
| `homeassistant/sensor/smartsafe_pro/boot_count/config` | HA discovery JSON | ✓ |
| `smartsafe/state` | `{"temp":24.5,"hum":55,"battery":78,"locked":true,"boot_count":12}` | ✓ |
| `smartsafe/events` | `{"event":"rfid_open","card":123456,"uptime":143}` | ✗ |
| `smartsafe/availability` | (reserved) | — |

### Event types

| `event` value | Trigger |
|---|---|
| `rfid_open` | Authorised RFID card presented |
| `web_open` | Remote unlock via web dashboard |
| `denied` | Unknown RFID card presented |

---

## State Machine

```
          ┌──────────┐
 Boot/    │          │  Touch / Web open
 Timeout  │   IDLE   │──────────────────► READER_ACTIVE
          │          │                         │
          └──────────┘                         │ Valid card / web token
               ▲                               ▼
               │                          LOCK_OPEN
               │                               │
               └───────── 3 s elapsed ─────────┘
```

- **IDLE** — 12 V boost and lock SSR are off; edit mode is cleared.
- **READER_ACTIVE** — 12 V boost enabled; Wiegand ISR active; times out after 15 s.
- **LOCK_OPEN** — SSR energised; 12 V boost off; auto-closes after 3 s.

**Edit mode** (card management) is entered by presenting the master card while in `READER_ACTIVE`. The next card scanned is registered if unknown, or deleted if it already exists.

---

## Power Management

The firmware implements a two-wake-reason model:

| Wake reason | Behaviour |
|---|---|
| **Boot** | Full initialisation — Wi-Fi, web server, RFID reader active |
| **Touch (EXT0, GPIO 32)** | Same as boot; immediately activates RFID reader |
| **Timer (1 h)** | Reads DHT11 + battery, connects Wi-Fi, flushes RTC buffer, publishes state to MQTT, **returns to sleep immediately** |

After 60 seconds of inactivity in boot/touch mode, the device:
1. Disconnects MQTT and Wi-Fi
2. Stops Bluetooth
3. Powers down GPIO outputs
4. Enters `esp_deep_sleep_start()`

---

## RTC Data Buffer

Up to **200 records** survive across deep-sleep cycles in RTC RAM (`RTC_DATA_ATTR`). Each record is 20 bytes:

```cpp
struct SensorRecord {
    uint32_t timestamp_s;  // millis()/1000 at recording time
    float    temp;
    float    hum;
    uint8_t  battery_pct;
    uint8_t  lock_state;   // 0=IDLE, 2=LOCK_OPEN
    uint8_t  event_type;   // 0=periodic, 1=rfid_open, 2=web_open, 3=denied
    uint8_t  reserved;
    uint32_t card_code;
};
// Total: 20 bytes × 200 entries = 4 000 bytes of RTC RAM
```

On the next successful MQTT connection all buffered records are flushed with a `"buffered":true` flag so Home Assistant history remains continuous.

---

## Building & Flashing

**Requirements:** PlatformIO Core ≥ 6.x or the PlatformIO IDE extension for VS Code.

```bash
# Install dependencies and build
pio run

# Flash to ESP32 on COM3 (adjust upload_port in platformio.ini)
pio run --target upload

# Open serial monitor at 115 200 baud
pio device monitor
```

The `platformio.ini` targets `esp32dev` board, Arduino framework, upload/monitor on `COM3` at 115 200 baud.

---

## Security Notes

1. **Credentials in source** — The repository template ships with placeholder strings. Always move real secrets to `include/secrets.h` and add that file to `.gitignore` before pushing.

2. **HTTP only** — The web dashboard uses plain HTTP. On an untrusted network, consider placing the device behind a reverse proxy with TLS, or restricting access to a trusted VLAN.

3. **HTTP Basic Auth** — Credentials are sent Base64-encoded (not encrypted) on every request. Use a strong, unique password.

4. **API token** — The `/open` endpoint validates a static bearer token. Rotate it regularly and never reuse it elsewhere.

5. **MQTT** — The broker should require authentication (already configured). Consider enabling TLS on the broker and switching `PubSubClient` to `WiFiClientSecure`.

6. **Master card code** — Treat the master card UID as a secret. Anyone who knows it (or physically has the card) can add or remove access cards.
