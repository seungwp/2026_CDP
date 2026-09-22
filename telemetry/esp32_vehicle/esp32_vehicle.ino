/*
 * esp32_vehicle.ino  -  차량용 ESP32 (A)
 *
 * 라즈베리파이에서 USB 시리얼로 차량 상태를 받아 BSM으로 ESP-NOW 방송한다.
 *
 * [시리얼 입력] 115200bps, 한 줄씩 '\n'으로 끝
 *     S,<danger_code>,<speed_mps>        예) S,2,0.15
 *     danger_code: 0=정상 1=급제동 2=비상정차(운전자 이상) 3=상태모름
 *   라즈베리파이는 초당 10회 계속 보낸다(하트비트).
 *   LINK_TIMEOUT_MS 동안 올바른 줄이 없으면 danger_code=3 으로 방송한다.
 *
 * [시리얼 출력] 1초마다 상태 한 줄 (라즈베리파이 로그용)
 *     A,<link>,<danger_code>,<speed>,<rx_count>,<bad_count>,<send_fail>
 *   '#'로 시작하는 줄은 사람이 읽는 안내문 (파서는 무시)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <stdlib.h>

// ===== 설정 =====
const uint32_t      MY_ID            = 1;       // 우리 차
const uint32_t      SERIAL_BAUD      = 115200;
const unsigned long LINK_TIMEOUT_MS  = 1000;    // 라즈베리파이 끊김 판정
const unsigned long TX_PERIOD_MS     = 100;     // 방송 주기 (초당 10회)
const unsigned long REPORT_PERIOD_MS = 1000;    // 상태 보고 주기
const uint8_t       CODE_MAX         = 3;
const uint8_t       CODE_UNKNOWN     = 3;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 화면용 ESP32와 반드시 같은 구조체 (크기가 다르면 수신 쪽에서 무시)
typedef struct {
  uint32_t vehicle_id;
  uint32_t timestamp;
  float    speed;         // m/s
  uint8_t  danger_code;   // 0=정상 1=급제동 2=비상정차 3=상태모름
} __attribute__((packed)) BSM;

BSM tx;

// ===== 시리얼 수신 상태 =====
char   lineBuf[64];
size_t lineLen      = 0;
bool   lineOverflow = false;

uint8_t       hostCode  = CODE_UNKNOWN;
float         hostSpeed = 0.0f;
bool          everRx    = false;
unsigned long lastRx    = 0;

unsigned long lastTx     = 0;
unsigned long lastReport = 0;
bool          lastLink   = false;

uint32_t rxCount  = 0;
uint32_t badCount = 0;
uint32_t sendFail = 0;

// "S,<code>,<speed>" 한 줄을 해석. 형식이 조금이라도 다르면 false.
bool parseLine(const char *s, uint8_t &code, float &speed) {
  if (s[0] != 'S' || s[1] != ',') return false;

  char *end;
  long c = strtol(s + 2, &end, 10);
  if (end == s + 2 || *end != ',') return false;
  if (c < 0 || c > CODE_MAX) return false;

  const char *p = end + 1;
  float v = strtof(p, &end);
  if (end == p) return false;
  while (*end == ' ' || *end == '\r') end++;
  if (*end != '\0') return false;
  if (isnan(v) || v < -10.0f || v > 10.0f) return false;   // 소형 차체 기준 비정상 값 거부

  code  = (uint8_t)c;
  speed = v;
  return true;
}

void readSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();

    if (ch == '\n') {
      if (lineOverflow) {
        badCount++;                       // 너무 긴 줄은 통째로 버림
      } else if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        uint8_t c;
        float v;
        if (parseLine(lineBuf, c, v)) {
          hostCode  = c;
          hostSpeed = v;
          lastRx    = millis();
          everRx    = true;
          rxCount++;
        } else {
          badCount++;
        }
      }
      lineLen      = 0;
      lineOverflow = false;
    } else if (!lineOverflow) {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = ch;
      } else {
        lineOverflow = true;
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  WiFi.mode(WIFI_STA);
  delay(200);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init 실패");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  tx.vehicle_id  = MY_ID;
  tx.speed       = 0.0f;
  tx.danger_code = CODE_UNKNOWN;

  Serial.printf("# esp32_vehicle ready  ID=%lu  MAC=%s\n",
                (unsigned long)MY_ID, WiFi.macAddress().c_str());
}

void loop() {
  readSerial();

  unsigned long now = millis();
  bool link = everRx && (now - lastRx <= LINK_TIMEOUT_MS);

  if (link != lastLink) {
    Serial.println(link ? "# link up (라즈베리파이 신호 수신)"
                        : "# link lost -> 코드 3(상태모름) 방송");
    lastLink = link;
  }

  if (now - lastTx >= TX_PERIOD_MS) {
    lastTx = now;
    tx.timestamp   = now;
    tx.danger_code = link ? hostCode : CODE_UNKNOWN;
    tx.speed       = link ? hostSpeed : 0.0f;
    if (esp_now_send(broadcastAddress, (uint8_t *)&tx, sizeof(tx)) != ESP_OK) {
      sendFail++;
    }
  }

  if (now - lastReport >= REPORT_PERIOD_MS) {
    lastReport = now;
    Serial.printf("A,%d,%u,%.2f,%lu,%lu,%lu\n",
                  link ? 1 : 0, tx.danger_code, tx.speed,
                  (unsigned long)rxCount, (unsigned long)badCount,
                  (unsigned long)sendFail);
  }

  delay(1);
}
