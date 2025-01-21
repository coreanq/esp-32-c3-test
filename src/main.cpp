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


bool isConnected = false;

// 함수 선언
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void deletePeer();

// 데이터 전송 콜백 함수
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// 데이터 수신 콜백 함수
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
    
    if( len > sizeof(myData.message)) {
        Serial.println("Data too long");
        return;
    }
    else {
        memcpy(&myData, incomingData, len);
        myData.message[len] = '\0';
        Serial.print(myData.message);
    }

    if( slave_peer_addr[0] == 0x00
        && slave_peer_addr[1] == 0x00
        && slave_peer_addr[2] == 0x00
        && slave_peer_addr[3] == 0x00
        && slave_peer_addr[4] == 0x00
        && slave_peer_addr[5] == 0x00 ){
        memcpy(slave_peer_addr, mac_addr, 6);
    }

    isConnected = true;
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

    do {
        if( c == -1) {
            break;
        }
        if( slave_peer_addr[0] == 0x00
            && slave_peer_addr[1] == 0x00
            && slave_peer_addr[2] == 0x00
            && slave_peer_addr[3] == 0x00
            && slave_peer_addr[4] == 0x00
            && slave_peer_addr[5] == 0x00 ){
                break;
        }

        esp_err_t result = esp_now_send(slave_peer_addr, (uint8_t *) &c, 1);
        Serial.print(slave_peer_addr[0], HEX);
        Serial.print(slave_peer_addr[1], HEX);
        Serial.print(slave_peer_addr[2], HEX);
        Serial.print(slave_peer_addr[3], HEX);
        Serial.print(slave_peer_addr[4], HEX);
        Serial.print(slave_peer_addr[5], HEX);
        Serial.println("");
        if (result == ESP_OK) {
            // Serial.print(c);
        } else {
            Serial.println("Failed");
            Serial.print( esp_err_to_name(result) );
            Serial.println("");
                
        }

    }while(false);
}

void setup() {
    // 시리얼 통신 초기화
    Serial.begin(921600);
    delay(1000);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    Serial.println("ESP-NOW Slave Start");
    
    // WiFi 모드 설정
    WiFi.mode(WIFI_AP);
    configDeviceAP();

    // This is the mac address of the Slave in AP Mode
    Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    // memcpy(slave_peer_addr, WiFi.softAPmacAddress().c_str(), ESP_NOW_ETH_ALEN);
    // Init ESPNow with a fallback logic
    InitESPNow();

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    static bool led_state = false;
    
    if( isConnected == true ) {
        sendData();
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
    }
    delay(10);
}
