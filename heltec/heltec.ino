#include <RadioLib.h>
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <HTTPClient.h>

// ---------- Pin definitions (Heltec V3) ----------
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_RST   12
#define LORA_BUSY  13
#define SPI_SCK    9
#define SPI_MISO   11
#define SPI_MOSI   10

// ---------- LoRa parameters ----------
#define LORA_FREQ   435.0
#define LORA_BW     125.0
#define LORA_SF     7
#define LORA_CR     5
#define LORA_SYNC   0x12

// ---------- SPLAT identities ----------
#define SAT_CALLSIGN  "CT6xxx"
#define GS_CALLSIGN   "CSXXXX"

// ---------- Command IDs ----------
#define CMD_FORCE_REBOOT       0
#define CMD_SWITCH_TO_STATE    2
#define CMD_UPLINK_TIME        3
#define CMD_REQUEST_TM_NOMINAL 7
#define CMD_REQUEST_TM_HAL     8
#define CMD_REQUEST_TM_STORAGE 9
#define CMD_RF_STOP            25
#define CMD_RF_RESUME          26

const char* ssid = "Pixel_7895";
const char* password = "lehotsPot";

// ---------- Display ----------
SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ---------- Radio ----------
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

volatile bool rxFlag = false;
void IRAM_ATTR rxISR() { rxFlag = true; }

// ---------- Big-endian read helpers ----------
uint8_t  ru8 (uint8_t* b) { return b[0]; }
int16_t  ri16(uint8_t* b) { return (int16_t)((b[0]<<8)|b[1]); }
uint16_t ru16(uint8_t* b) { return (uint16_t)((b[0]<<8)|b[1]); }
uint32_t ru32(uint8_t* b) { return (uint32_t)((b[0]<<24)|(b[1]<<16)|(b[2]<<8)|b[3]); }
int32_t  ri32(uint8_t* b) { return (int32_t)((b[0]<<24)|(b[1]<<16)|(b[2]<<8)|b[3]); }
float    rf32(uint8_t* b) { uint32_t u=ru32(b); float f; memcpy(&f,&u,4); return f; }

// ---------- Big-endian write helpers ----------
void wu32(uint8_t* b, uint32_t v) { b[0]=v>>24; b[1]=v>>16; b[2]=v>>8; b[3]=v; }
void wu8 (uint8_t* b, uint8_t  v) { b[0]=v; }

// ---------- Rolling display buffer ----------
String lines[5];
void pushLine(String s) {
  for (int i = 0; i < 4; i++) lines[i] = lines[i+1];
  lines[4] = s;
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  for (int i = 0; i < 5; i++) oled.drawString(0, i*12, lines[i]);
  oled.display();
}

// ---------- Build and send a no-arg command ----------
bool sendCommand(uint8_t cmd_id) {
  uint8_t pkt[8];
  memcpy(pkt, GS_CALLSIGN, 6);
  pkt[6] = (0x02 << 5) | ((cmd_id >> 8) & 0x1F);
  pkt[7] = cmd_id & 0xFF;
  int state = radio.transmit(pkt, 8);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] CMD %u sent OK\n", cmd_id);
    return true;
  } else {
    Serial.printf("[TX] CMD %u FAIL: %d\n", cmd_id, state);
    return false;
  }
}

// ---------- Build and send SWITCH_TO_STATE ----------
bool sendSwitchState(uint8_t state_id, uint32_t time_in_state) {
  uint8_t pkt[13];
  memcpy(pkt, GS_CALLSIGN, 6);
  pkt[6] = (0x02 << 5) | ((CMD_SWITCH_TO_STATE >> 8) & 0x1F);
  pkt[7] = CMD_SWITCH_TO_STATE & 0xFF;
  wu8 (pkt+8, state_id);
  wu32(pkt+9, time_in_state);
  int state = radio.transmit(pkt, 13);
  return (state == RADIOLIB_ERR_NONE);
}

// ---------- Build and send UPLINK_TIME_REFERENCE ----------
bool sendUplinkTime(uint32_t unix_time) {
  uint8_t pkt[12];
  memcpy(pkt, GS_CALLSIGN, 6);
  pkt[6] = (0x02 << 5) | ((CMD_UPLINK_TIME >> 8) & 0x1F);
  pkt[7] = CMD_UPLINK_TIME & 0xFF;
  wu32(pkt+8, unix_time);
  int state = radio.transmit(pkt, 12);
  return (state == RADIOLIB_ERR_NONE);
}

// ---------- Decode TM_HEARTBEAT ----------
void decodeHeartbeat(uint8_t* buf, int len) {
  if (len < 7) return;
  uint8_t* d = buf + 7;  // skip 6B callsign + 1B header
  int rem    = len - 7;

  Serial.println("=== TM_HEARTBEAT ===");

  // --- CDH: TIME(4) SC_STATE(1) SD_USAGE(4) RAM(1) REBOOT(1) WDT(1) HAL(1) DETUMBLE(1) = 14B ---
  if (rem < 14) { Serial.println("Too short: CDH"); return; }
  uint32_t timestamp = ru32(d);
  uint8_t  sc_state  = ru8 (d+4);
  uint32_t sd_usage  = ru32(d+5);
  uint8_t  ram       = ru8 (d+9);
  uint8_t  reboots   = ru8 (d+10);
  uint8_t  wdt       = ru8 (d+11);
  uint8_t  hal_flags = ru8 (d+12);
  uint8_t  detumble  = ru8 (d+13);
  Serial.printf("TIME:%u SC_STATE:%u SD:%uKB RAM:%u%% REBOOTS:%u WDT:%u HAL:0x%02X DET:%u\n",
                timestamp, sc_state, sd_usage, ram, reboots, wdt, hal_flags, detumble);
  pushLine("T:" + String(timestamp) + " ST:" + String(sc_state));
  pushLine("RAM:" + String(ram) + "% RBT:" + String(reboots));
  d += 14; rem -= 14;

  // --- EPS: 86 bytes total ---
  // EPS_POWER_FLAG(1) MB_TEMP(2) MB_VOLT(2) MB_CURR(2)          = 7
  // BATT_TEMP(2) BATT_SOC(1) BATT_CAP(2) BATT_CURR(2) BATT_V(2) = 9
  // BATT_MIDPOINT_V(2) TTE(4) TTF(4)                            = 10
  // 6 coil pairs V+I h+h = 6*4                                  = 24
  // JETSON V+I(4) RF V+I(4) GPS V+I(4)                          = 12
  // 6 solar pairs V+I h+h = 6*4                                  = 24
  // Total = 7+9+10+24+12+24 = 86B
  if (rem < 3) { Serial.println("Too short: EPS"); return; }
  float mb_temp = ri16(d+1) * 0.1f;
  Serial.printf("MB_TEMP: %.1f C\n", mb_temp);
  pushLine("MB:" + String(mb_temp, 1) + "C");
  d += 86; rem -= 86;  // skip full EPS block

  // --- ADCS ---
  if (rem < 1) { Serial.println("Too short: ADCS MODE"); return; }
  uint8_t adcs_mode = ru8(d);
  Serial.printf("ADCS MODE:%u\n", adcs_mode);
  d += 1; rem -= 1;

  // GYRO x/y/z = 12B
  if (rem < 12) { Serial.println("Too short: GYRO"); return; }
  float gx = rf32(d), gy = rf32(d+4), gz = rf32(d+8);
  Serial.printf("GYRO: %.4f %.4f %.4f rad/s\n", gx, gy, gz);
  pushLine("GX:" + String(gx,3) + " GY:" + String(gy,3));
  d += 12; rem -= 12;

  // MAG x/y/z = 12B
  if (rem < 12) { Serial.println("Too short: MAG"); return; }
  float mx = rf32(d), my = rf32(d+4), mz = rf32(d+8);
  Serial.printf("MAG: %.3e %.3e %.3e T\n", mx, my, mz);
  d += 12; rem -= 12;

  // SUN_STATUS(1) + SUN_VEC(12) + LIGHT_SENSORS 9*2(18) = 31B — all zeros, skip
  d += 31; rem -= 31;

  // COIL STATUS x6 = 6B
  if (rem < 6) { Serial.println("Too short: COIL"); return; }
  Serial.printf("COIL: XP=%u XM=%u YP=%u YM=%u ZP=%u ZM=%u\n",
                ru8(d), ru8(d+1), ru8(d+2), ru8(d+3), ru8(d+4), ru8(d+5));
  pushLine("COIL XP:" + String(ru8(d)) + " YP:" + String(ru8(d+2)));
  d += 6; rem -= 6;

  // GPS — 15 fields, all zeros without fix, print anyway
  // GPS_MSG_ID(1) FIX_MODE(1) NUM_SV(1) WEEK(2) TOW(4) = 9B
  if (rem < 9) { Serial.println("Too short: GPS"); return; }
  uint8_t  fix_mode = ru8 (d+1);
  uint8_t  num_sv   = ru8 (d+2);
  uint32_t tow      = ru32(d+5);
  Serial.printf("GPS FIX:%u SV:%u TOW:%u\n", fix_mode, num_sv, tow);
  d += 9; rem -= 9;

  // LAT(4) LON(4) ALT(4) = 12B
  if (rem < 12) { Serial.println("Too short: GPS pos"); return; }
  float lat = ri32(d)   * 1e-7f;
  float lon = ri32(d+4) * 1e-7f;
  float alt = ri32(d+8) * 0.01f;
  Serial.printf("LAT:%.6f LON:%.6f ALT:%.1fm\n", lat, lon, alt);
  if (fix_mode > 0)
    pushLine("LAT:" + String(lat,4) + " LON:" + String(lon,4));
}

bool sendEvalCommand(const char* expr) {
  int exprLen = strlen(expr);
  int pktLen  = 8 + exprLen;  // 6B callsign + 2B header + string
  uint8_t pkt[256];
  memcpy(pkt, GS_CALLSIGN, 6);
  pkt[6] = (0x02 << 5) | ((15 >> 8) & 0x1F);  // cmd_id=15
  pkt[7] = 15 & 0xFF;
  memcpy(pkt+8, expr, exprLen);
  int state = radio.transmit(pkt, pktLen);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] EVAL sent: %s\n", expr);
    return true;
  } else {
    Serial.printf("[TX] EVAL FAIL: %d\n", state);
    return false;
  }
}

void postTemperature(float temperature) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] Not connected, skipping POST");
    return;
  }

  HTTPClient http;
  http.begin("http://35.222.71.40:8080");
  http.addHeader("Content-Type", "text/plain");

  String body = String(temperature, 2);

  int httpCode = http.POST(body);

  if (httpCode > 0) {
    Serial.printf("[HTTP] POST %d — body: %s\n", httpCode, body.c_str());
    pushLine("POST OK " + String(httpCode));
  } else {
    Serial.printf("[HTTP] POST failed: %s\n", http.errorToString(httpCode).c_str());
    pushLine("POST FAIL");
  }

  http.end();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);

  oled.init();
  oled.flipScreenVertically();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "GS Init...");
  oled.display();

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LORA_NSS);

  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, 10);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Radio OK");
    pushLine("Radio OK");
  } else {
    Serial.printf("Radio FAIL: %d\n", state);
    pushLine("Radio FAIL: " + String(state));
    while (true);
  }

  radio.setPreambleLength(8);
  radio.setTCXO(1.7);
  radio.setDio1Action(rxISR);
  radio.startReceive();

  pushLine(String(LORA_FREQ,1) + "MHz SF" + String(LORA_SF));
  pushLine("GS Listening...");

  pushLine("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  pushLine("WiFi Connected!");
}

// ---------- Loop ----------
void loop() {
  // --- Serial command interface ---
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'h':
        Serial.println("Commands: h=heartbeat r=reboot t=uplink_time");
        Serial.println("          1=LED red 2=LED green 3=LED blue 0=LED off");
        break;
      case 'r':
        Serial.println("Sending FORCE_REBOOT...");
        sendCommand(CMD_FORCE_REBOOT);
        break;
      case 't':
        Serial.println("Sending UPLINK_TIME...");
        sendUplinkTime((uint32_t)(millis()/1000));
        break;
      case '1':
        sendEvalCommand(
          "SATELLITE.NEOPIXEL.fill((255,0,0));import time;time.sleep(2);SATELLITE.NEOPIXEL.fill((0,0,0))"
        );
        break;
      case '2':
        sendEvalCommand(
          "SATELLITE.NEOPIXEL.fill((0,255,0));import time;time.sleep(2);SATELLITE.NEOPIXEL.fill((0,0,0))"
        );
        break;
      case '3':
        sendEvalCommand(
          "SATELLITE.NEOPIXEL.fill((0,0,255));import time;time.sleep(2);SATELLITE.NEOPIXEL.fill((0,0,0))"
        );
        break;
      case '0':
        sendEvalCommand("SATELLITE.NEOPIXEL.fill((0,0,0))");
        break;
    }
  }

  if (!rxFlag) return;
  rxFlag = false;

  uint8_t buf[256];
  int state = radio.readData(buf, sizeof(buf));
  int len   = radio.getPacketLength();

  if (state == RADIOLIB_ERR_NONE) {
    float rssi = radio.getRSSI();
    float snr  = radio.getSNR();

    Serial.printf("[RX] %d bytes  RSSI:%.0f dBm  SNR:%.1f dB\n", len, rssi, snr);
    for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();

    float lattitude = 0.0f;
    float longitude = 0.0f;
    float altitude = 0.0f;
    float temperature = 0.0f;
    float pressure = 0.0f;
    float humidity = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    memcpy(&lattitude, buf, 4);
    memcpy(&longitude, buf + 4, 8);
    memcpy(&altitude, buf + 8, 8);
    memcpy(&temperature, buf + 12, 8);

    Serial.printf("Temperature: %.2f\n", temperature);
    Serial.println();

    postTemperature(temperature);

    if (len >= 7) {
      bool fromSat = (memcmp(buf, SAT_CALLSIGN, 6) == 0);
      bool fromGS  = (memcmp(buf, GS_CALLSIGN,  6) == 0);

      if (fromSat || fromGS) {
        uint8_t hdr     = buf[6];
        uint8_t msgType = (hdr >> 5) & 0x07;
        uint8_t subId   = hdr & 0x1F;

        char cs[7] = {0};
        memcpy(cs, buf, 6);
        Serial.printf("From:%s Type:%u ID:%u\n", cs, msgType, subId);

        if (msgType == 0 && subId == 1) {
          // TM_HEARTBEAT — decode then reply immediately
          decodeHeartbeat(buf, len);
          pushLine("Sending CMD...");
          bool ok = sendCommand(CMD_REQUEST_TM_NOMINAL);
          pushLine(ok ? "CMD sent OK" : "CMD FAIL");

        } else if (msgType == 0) {
          // Other report type
          Serial.printf("Report ID:%u  %d payload bytes\n", subId, len-7);
          pushLine("RPT ID:" + String(subId) + " " + String(len-7) + "B");

        } else if (msgType == 6) {
          // ACK
          uint8_t status = hdr & 0x1F;
          Serial.printf("ACK status:%u\n", status);
          pushLine("ACK status:" + String(status));
          if (len > 7) {
            String msg = "";
            for (int i = 7; i < len; i++) msg += (char)buf[i];
            Serial.println("ACK: " + msg);
            pushLine(msg.substring(0, 21));
          }

        } else {
          Serial.printf("Unhandled type:%u id:%u\n", msgType, subId);
          pushLine("Type:" + String(msgType) + " ID:" + String(subId));
        }

      } else {
        char cs[7] = {0};
        memcpy(cs, buf, 6);
        Serial.printf("Unknown callsign: %s\n", cs);
      }
    }

    pushLine("R:" + String((int)rssi) + " S:" + String(snr,1));

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[RX] CRC error");
    pushLine("CRC err");
  } else {
    Serial.printf("[RX] ERR %d\n", state);
    pushLine("RX err: " + String(state));
  }

  radio.startReceive();
}