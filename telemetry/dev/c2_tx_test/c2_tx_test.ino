// C2 하드웨어 확인용 (차량용 ESP32-A 자리에 올린다)
// 위험코드를 0 -> 1 -> 2 -> 3 순서로 2초마다 바꿔서 ESP-NOW로 방송한다.
// 화면용 ESP32-B(esp-recive.ino)가 받으면 넥션 page 0~3이 2초마다 돌아가야 정상.

#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 기존 코드와 반드시 같은 구조체여야 한다 (크기가 다르면 수신 쪽에서 무시함)
typedef struct {
  uint32_t vehicle_id;
  uint32_t timestamp;
  float    speed;
  uint8_t  danger_code;   // 0=정상 1=급제동 2=비상정차 3=신호없음/상태모름
} __attribute__((packed)) BSM;

const uint32_t MY_ID = 1;   // 우리 차 = 1

BSM tx;
unsigned long lastChange = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(500);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init 실패");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  tx.vehicle_id  = MY_ID;
  tx.speed       = 0.0;
  tx.danger_code = 0;
  Serial.println("C2 테스트 시작: 2초마다 위험코드 변경");
}

void loop() {
  if (millis() - lastChange >= 2000) {
    lastChange = millis();
    tx.danger_code = (tx.danger_code + 1) % 4;
    Serial.printf("방송 위험코드 -> %u\n", tx.danger_code);
  }

  tx.timestamp = millis();
  esp_now_send(broadcastAddress, (uint8_t *)&tx, sizeof(tx));
  delay(100);   // 초당 10회
}
