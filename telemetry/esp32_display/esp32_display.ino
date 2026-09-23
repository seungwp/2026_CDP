/*
 * esp32_display.ino  -  화면용 ESP32 (B), 뒤차 운전석 화면
 *
 * 우리 차(차량용 ESP32-A, ID=1)가 ESP-NOW로 방송하는 BSM을 받아
 * 위험코드에 맞는 넥션 페이지를 띄운다.
 *
 *   danger_code 0 -> page 0  이상없음
 *   danger_code 1 -> page 1  전방차량 급제동
 *   danger_code 2 -> page 2  전방차량 비상정차 (운전자 이상)
 *   danger_code 3 -> page 3  전방차량 신호없음 (차량 상태 모름)
 *   LINK_TIMEOUT_MS 동안 ID=1 신호가 없음 -> page 3
 *
 * 넥션: UART2, 9600bps, RX=16 TX=17
 * 보드 LED(GPIO 2): 수신 중 켜짐, 끊기면 깜빡임
 * 시리얼(115200): 페이지 바뀔 때 + 1초마다 상태 한 줄
 *     B,<link>,<page>,<rx_count>,<ignored>
 */

#include <WiFi.h>
#include <esp_now.h>

// ===== 설정 =====
const uint32_t      TARGET_ID       = 1;      // 우리 차 ID만 반영
const unsigned long LINK_TIMEOUT_MS = 1000;   // 이 시간 못 받으면 신호없음
const uint8_t       PAGE_NO_SIGNAL  = 3;
const uint8_t       CODE_MAX        = 3;
const int           LED_PIN         = 2;      // 보드 LED가 없는 모델이면 영향 없음
const unsigned long REPORT_MS       = 1000;

// 차량용 ESP32와 반드시 같은 구조체
typedef struct {
  uint32_t vehicle_id;
  uint32_t timestamp;
  float    speed;
  uint8_t  danger_code;
} __attribute__((packed)) BSM;

HardwareSerial NextionSerial(2);

// ===== 수신 콜백이 채우는 값 (무선 처리 쪽에서 쓰고 loop에서 읽음) =====
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t       rxCode    = PAGE_NO_SIGNAL;
volatile float         rxSpeed   = 0.0f;
volatile unsigned long rxTime    = 0;
volatile bool          rxEver    = false;
volatile uint32_t      rxCount   = 0;
volatile uint32_t      rxIgnored = 0;

// ===== loop 상태 =====
uint8_t       shownPage  = 255;   // 아직 아무 페이지도 안 띄움
bool          lastLink   = false;
unsigned long lastReport = 0;

void nextionSetPage(uint8_t pageNum) {
  NextionSerial.print("page ");
  NextionSerial.print(pageNum);
  NextionSerial.write(0xFF);
  NextionSerial.write(0xFF);
  NextionSerial.write(0xFF);
}

// 무선 처리 중에 불리므로 값만 저장하고 바로 나온다 (넥션 통신은 loop에서)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(BSM)) {
    portENTER_CRITICAL(&rxMux);
    rxIgnored++;
    portEXIT_CRITICAL(&rxMux);
    return;
  }
  BSM rx;
  memcpy(&rx, data, sizeof(BSM));

  portENTER_CRITICAL(&rxMux);
  if (rx.vehicle_id != TARGET_ID || rx.danger_code > CODE_MAX) {
    rxIgnored++;
  } else {
    rxCode  = rx.danger_code;
    rxSpeed = rx.speed;
    rxTime  = millis();
    rxEver  = true;
    rxCount++;
  }
  portEXIT_CRITICAL(&rxMux);
}

void setup() {
  Serial.begin(115200);
  NextionSerial.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(500);

  WiFi.mode(WIFI_STA);
  delay(200);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init 실패");
    return;
  }
  esp_now_register_recv_cb(onRecv);

  Serial.printf("# esp32_display ready  target ID=%lu  MAC=%s\n",
                (unsigned long)TARGET_ID, WiFi.macAddress().c_str());
}

void loop() {
  unsigned long now = millis();

  uint8_t       code;
  unsigned long t;
  bool          ever;
  uint32_t      cnt, ign;
  portENTER_CRITICAL(&rxMux);
  code = rxCode;
  t    = rxTime;
  ever = rxEver;
  cnt  = rxCount;
  ign  = rxIgnored;
  portEXIT_CRITICAL(&rxMux);

  bool link = ever && (now - t <= LINK_TIMEOUT_MS);
  uint8_t page = link ? code : PAGE_NO_SIGNAL;

  if (link != lastLink) {
    Serial.println(link ? "# 우리 차 신호 수신 시작"
                        : "# 우리 차 신호 끊김 -> page 3 (신호없음)");
    lastLink = link;
  }

  if (page != shownPage) {
    nextionSetPage(page);
    Serial.printf("# 넥션 page %u -> %u\n", shownPage, page);
    shownPage = page;
  }

  // LED: 수신 중 켜짐 / 끊김 깜빡임 (0.25초 간격)
  digitalWrite(LED_PIN, link ? HIGH : ((now / 250) % 2 ? HIGH : LOW));

  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    Serial.printf("B,%d,%u,%lu,%lu\n", link ? 1 : 0, shownPage,
                  (unsigned long)cnt, (unsigned long)ign);
  }

  delay(5);
}
