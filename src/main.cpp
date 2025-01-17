#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ESP-NOW 슬레이브 정보를 저장할 전역 변수
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 0

// 데이터 전송을 위한 구조체 정의
typedef struct struct_message {
    char message[512];
} struct_message;
struct_message myData;

// 함수 선언
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void deletePeer();

// 데이터 전송 콜백 함수
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 데이터 수신 콜백 함수
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("recv data : ");
    Serial.println(myData.message);
}

// ESP-NOW 초기화 함수
void InitESPNow() {
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        Serial.println("ESPNow Init Success");
    } else {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP("Slave_1", "kcpark", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}


// 데이터 전송 함수
void sendData() {
    const uint8_t *peer_addr = slave.peer_addr;
    if (Serial1.available() > 0) {
        int len = Serial.readBytes(myData.message, 512);
        esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &myData, len);
        
        Serial.print("Send Status: ");
        if (result == ESP_OK) {
            Serial.println("Success");
        } else {
            Serial.println("Failed");
        }
    }
}

void setup() {
    // 시리얼 통신 초기화
    Serial.begin(921600);
    delay(1000);
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
    
    Serial.println("ESP-NOW Slave Start");
    
    // WiFi 모드 설정
    WiFi.mode(WIFI_AP);
    configDeviceAP();

    // This is the mac address of the Slave in AP Mode
    Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    // Init ESPNow with a fallback logic
    InitESPNow();

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    Serial.println("loop");
    delay(1000);
}