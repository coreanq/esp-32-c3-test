#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define LED_PIN 8


uint8_t slave_peer_addr[ESP_NOW_ETH_ALEN] = {0x00, };

#define CHANNEL 1
#define PRINTSCANRESULTS 3
#define DELETEBEFOREPAIR 0

// 데이터 전송을 위한 구조체 정의
typedef struct struct_message {
    char message[512];
} struct_message;
struct_message myData;


bool isConnected = false;

// 함수 선언
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// 데이터 전송 콜백 함수
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// 데이터 수신 콜백 함수
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) 
{
    if( isConnected == false ) {
        Serial.print("Peer MAC : ");
        Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", 
            esp_now_info->src_addr[0], 
            esp_now_info->src_addr[1], 
            esp_now_info->src_addr[2], 
            esp_now_info->src_addr[3], 
            esp_now_info->src_addr[4], 
            esp_now_info->src_addr[5]);

        memcpy(slave_peer_addr, esp_now_info->src_addr, 6);
        slave_peer_addr[5] = slave_peer_addr[5] + 1;
    }
    
    if( data_len > sizeof(myData.message)) {
        Serial.println("Data too long");
        return;
    }
    else {
        memcpy(&myData, data, data_len);
        myData.message[data_len] = '\0';
        Serial.print(myData.message);
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
    
    char buffer[512];
    int len = Serial.readBytes(buffer, 512);

    do {
        if( len == -1 || len == 0 ) {
            break;
        }
        
        if( isConnected == false ) {
            break;
        }

        esp_err_t result = esp_now_send(slave_peer_addr, (uint8_t *) buffer, len);
        
        Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", slave_peer_addr[0], slave_peer_addr[1], slave_peer_addr[2], slave_peer_addr[3], slave_peer_addr[4], slave_peer_addr[5]);
        Serial.println("");
        if (result == ESP_OK) {
            // Serial.print(c);
        } else {
            Serial.println("Failed");
            Serial.print( "len : " );
            Serial.print( len );
            Serial.print( esp_err_to_name(result) );
            Serial.println("");
                
        }

    }while(false);
}

void setup() {
    // 시리얼 통신 초기화
    Serial.setTimeout(1);
    Serial.begin(921600);
    delay(1000);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.print("Date : ");
    Serial.println(__DATE__);
    Serial.print("Time : ");
    Serial.println(__TIME__);
    Serial.println("ESP-NOW Slave Hello");
    
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
    static bool led_state = false;
    
    if( isConnected == true ) {
        sendData();
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
    }
    delay(10);
}
