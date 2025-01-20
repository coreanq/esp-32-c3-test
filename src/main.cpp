#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define LED_PIN 8


uint8_t slave_peer_addr[ESP_NOW_ETH_ALEN];

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
}

// 데이터 수신 콜백 함수
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    
    if( len > sizeof(myData.message)) {
        Serial.println("Data too long");
        return;
    }
    else {
        memcpy(&myData, incomingData, sizeof(myData));
        myData.message[len] = '\0';
        Serial.print(myData.message);
    }
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
  bool result = WiFi.softAP("Slave_1", "kcpark1234", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}


// 데이터 전송 함수
void sendData() {
    int c = Serial.read();
    if( c != -1) {
        Serial.print((char)c);
        esp_err_t result = esp_now_send(slave_peer_addr, (uint8_t *) &c, 1);
        if (result == ESP_OK) {
            // Serial.print(c);
        } else {
            // Serial.println("Failed");
        }
    }
    else{
        // Serial.println("No data");
    }
    // if (Serial.available() > 0) {
    //     int len = Serial.readBytes(myData.message, 512);
    //     esp_err_t result = esp_now_send(slave_peer_addr, (uint8_t *) &myData, len);
    //     myData.message[len] = '\0';
    //     Serial.println(myData.message);
    //     if (result == ESP_OK) {
    //         Serial.print(myData.message);
    //     } else {
    //         Serial.println("Failed");
    //     }
    // }
}

void setup() {
    // 시리얼 통신 초기화
    Serial.begin(921600);
    delay(1000);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("ESP-NOW Slave Start");
    
    // WiFi 모드 설정
    WiFi.mode(WIFI_AP);
    configDeviceAP();

    // This is the mac address of the Slave in AP Mode
    Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    memcpy(slave_peer_addr, WiFi.softAPmacAddress().c_str(), ESP_NOW_ETH_ALEN);
    // Init ESPNow with a fallback logic
    InitESPNow();

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    static bool led_state = false;
    
    if( esp_now_is_peer_exist(slave_peer_addr) == true ) {
        // Serial.println("loop");
        // digitalWrite(LED_PIN, HIGH);
        // delay(500);
        // digitalWrite(LED_PIN, LOW);
        // delay(500);
        sendData();

        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
    }
    delay(10);
}
