#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ===== BSM: 실제 차량 안전 메시지를 본뜬 구조체 =====
typedef struct {
  uint32_t vehicle_id;
  uint32_t timestamp;
  float    speed;
  uint8_t  danger_code;  // 0=정상, 1=전방감지, 2=후방감지
} __attribute__((packed)) BSM;

const uint32_t MY_ID = 2;

BSM tx;
HardwareSerial NextionSerial(2);   // UART2를 넥션용으로

uint8_t lastPage = 255;   // 마지막으로 띄운 페이지 (처음엔 없음 표시)

// ===== 넥션에 페이지 전환 명령 보내기 =====
void nextionSetPage(uint8_t pageNum) {
  NextionSerial.print("page ");
  NextionSerial.print(pageNum);
  NextionSerial.write(0xFF);   // 종료 신호 3번
  NextionSerial.write(0xFF);
  NextionSerial.write(0xFF);
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(BSM)) return;
  BSM rx;
  memcpy(&rx, data, sizeof(BSM));
  Serial.printf("수신 | ID:%lu  time:%lu  speed:%.1f  danger:%u\n",
    rx.vehicle_id, rx.timestamp, rx.speed, rx.danger_code);

  // 위험코드에 맞는 페이지로 전환 (값이 바뀐 순간만)
  if (rx.danger_code != lastPage) {
    nextionSetPage(rx.danger_code);
    lastPage = rx.danger_code;
    Serial.printf(">> 넥션 페이지 전환: page %u\n", rx.danger_code);
  }
}

void setup() {
  Serial.begin(115200);
  NextionSerial.begin(9600, SERIAL_8N1, 16, 17);  // 넥션: RX=16, TX=17
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(500);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init 실패");
    return;
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  tx.vehicle_id  = MY_ID;
  tx.speed       = 60.0;
  tx.danger_code = 0;
}

void loop() {
  tx.timestamp = millis();
  esp_now_send(broadcastAddress, (uint8_t*)&tx, sizeof(tx));
  delay(100);
}
