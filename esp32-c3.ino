#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define LED_PIN 8
#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 0
#define DEBUG_MSG_BUFFER_SIZE 4096

#define BYPASS_SRC_PORT     Serial0
#define DEBUG_PORT          Serial1

#define RS485_TX_ENABLE_PIN 5

#define ESPNOW_DEBUG

// 데이터 전송을 위한 구조체 정의
typedef struct struct_message {
    char message[DEBUG_MSG_BUFFER_SIZE];
} struct_message;


struct_message myData;
struct_message bypassSerialData; // serial Task 에서만 사용

bool isConnected = false;

QueueHandle_t msgRecv485Queue;
QueueHandle_t msgSend485Queue;

// ESP-NOW 슬레이브 정보를 저장할 전역 변수
// Send 시 반드시 필요 
esp_now_peer_info_t slave = {
    .peer_addr = {0},
    .channel = CHANNEL,
    .ifidx = WIFI_IF_AP,
    .encrypt = false
};

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
        DEBUG_PORT.print("Peer MAC : ");
        DEBUG_PORT.printf("%02X:%02X:%02X:%02X:%02X:%02X", 
            esp_now_info->src_addr[0], 
            esp_now_info->src_addr[1], 
            esp_now_info->src_addr[2], 
            esp_now_info->src_addr[3], 
            esp_now_info->src_addr[4], 
            esp_now_info->src_addr[5]);

        memcpy(slave.peer_addr, esp_now_info->src_addr, 6);
        slave.channel = CHANNEL;

        // 페어링 상태 확인
        DEBUG_PORT.print("Slave Status: ");
        bool exists = esp_now_is_peer_exist(slave.peer_addr);
        if (exists) {
            DEBUG_PORT.println("Already Paired");
        } else {
            // 새로운 페어링 시도
            esp_err_t addStatus = esp_now_add_peer(&slave);
            if (addStatus == ESP_OK) {
                DEBUG_PORT.println("Pair success");
            } else {
                DEBUG_PORT.println("Pair failed");
            }
        }
    }
        
    if( data_len > sizeof(myData.message)) {
        DEBUG_PORT.println("Data too long");
        return;
    }
    else {
        memcpy(&myData, data, data_len);
        myData.message[data_len] = '\0';
        DEBUG_PORT.print(myData.message);

        for(int i = 0; i < data_len; i++) {
            xQueueSend( msgSend485Queue, &myData.message[i], 0 );
        }
    }

    isConnected = true;
    
}

// ESP-NOW 초기화 함수
void InitESPNow() {
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        DEBUG_PORT.println("ESPNow Init Success");
    } else {
        DEBUG_PORT.println("ESPNow Init Failed");
        ESP.restart();
    }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP("Slave_1", "kcpark1234", CHANNEL, 0);
  if (!result) {
    DEBUG_PORT.println("AP Config failed.");
  } else {
    DEBUG_PORT.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    DEBUG_PORT.print("AP CHANNEL "); DEBUG_PORT.println(WiFi.channel());
  }
}



// 데이터 전송 함수
void sendDataToWifi() {
    int ItemCount = 0;
    do {
        ItemCount = uxQueueMessagesWaiting( msgRecv485Queue );
            
        if( ItemCount == 0 ){
            break;
        }

        if( ItemCount > 200 ) {
            // esp now packet size 256
            ItemCount = 200;
        }

        for( int i = 0; i < ItemCount; i++ ) {
            xQueueReceive( msgRecv485Queue, &myData.message[i],  0 );
        }

        esp_err_t result = esp_now_send(slave.peer_addr, (uint8_t *) myData.message, ItemCount);
        
        if (result == ESP_OK) {
            myData.message[ItemCount] = '\0';
            DEBUG_PORT.print("send data to wifi: ");

            for( int i = 0; i < ItemCount; i++ ) {
                DEBUG_PORT.print(myData.message[i], HEX);
                DEBUG_PORT.print(" ");
            }
            DEBUG_PORT.println("");

        } else {
            DEBUG_PORT.println("Failed");
            DEBUG_PORT.print( "len : " );
            DEBUG_PORT.println( ItemCount );
            DEBUG_PORT.print( esp_err_to_name(result) );
            DEBUG_PORT.println("");
        }

    }while(false);
}        
        


void bypssSerialTask(void* parameter) 
{
    int sendLen = 0;
    int recvLen = 0;

    while( true )
    {
        recvLen = BYPASS_SRC_PORT.readBytes(bypassSerialData.message, DEBUG_MSG_BUFFER_SIZE);
        bypassSerialData.message[recvLen] = '\0';
        
        if( recvLen > 0 ) {
            for ( int i = 0; i < recvLen; i++ ) {
                xQueueSend( msgRecv485Queue, &bypassSerialData.message[i], 0 );
            }
            DEBUG_PORT.print("recv data from 485: ");
            
            for(int i = 0; i < recvLen; i++) {
                DEBUG_PORT.print(bypassSerialData.message[i], HEX);
                DEBUG_PORT.print(" ");
            }
            DEBUG_PORT.println("");
        }
        else {
            // DEBUG_PORT.println("No data to send");
        }

        do{
            sendLen = uxQueueMessagesWaiting( msgSend485Queue );
            if( sendLen == 0 ) {
                vTaskDelay( portTICK_PERIOD_MS);
                break;
            }

            if( sendLen > 200 ) {
                sendLen = 200;
            }
            DEBUG_PORT.print("send data to 485:  ");
            DEBUG_PORT.println(sendLen);
            
            for( int i = 0; i < sendLen; i++ ) {
                xQueueReceive( msgSend485Queue, &bypassSerialData.message[i], 0 );
            }
            
            digitalWrite(RS485_TX_ENABLE_PIN, HIGH);
            vTaskDelay( portTICK_PERIOD_MS);

            BYPASS_SRC_PORT.write(bypassSerialData.message, sendLen);

            digitalWrite(RS485_TX_ENABLE_PIN, LOW);
            vTaskDelay( portTICK_PERIOD_MS);

        }while(false);

    }


}

void setup() {
    // 시리얼 통신 초기화, before begin
    
    BYPASS_SRC_PORT.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    BYPASS_SRC_PORT.setTimeout(1);

    DEBUG_PORT.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    DEBUG_PORT.setTimeout(1);


    Serial.begin(3000000);  // USB to Serial

    BYPASS_SRC_PORT.begin(460800, SERIAL_8N1, 20, 21); // rx 20, tx 21
    DEBUG_PORT.begin(3000000, SERIAL_8N1, 1, 0); // rx 0, tx 1
    
    pinMode(RS485_TX_ENABLE_PIN, OUTPUT);
    digitalWrite(RS485_TX_ENABLE_PIN, LOW); // DE HIGH 송신 (tx) 활성화 

    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    DEBUG_PORT.print("\n\n-------------------------------------------------------------------\n");
    DEBUG_PORT.print("Current Date : ");
    DEBUG_PORT.println(__DATE__);
    DEBUG_PORT.print("Time : ");
    DEBUG_PORT.println(__TIME__);
    
    // WiFi 모드 설정
    WiFi.mode(WIFI_AP);
    configDeviceAP();
    
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    // This is the mac address of the Slave in AP Mode
    DEBUG_PORT.print("AP MAC Address: "); 
    DEBUG_PORT.println(WiFi.softAPmacAddress());

    // Init ESPNow with a fallback logic
    InitESPNow();

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Queue 생성
    msgRecv485Queue = xQueueCreate( DEBUG_MSG_BUFFER_SIZE, sizeof(char) );
    msgSend485Queue = xQueueCreate( DEBUG_MSG_BUFFER_SIZE, sizeof(char) );

    xTaskCreate(bypssSerialTask, "bypssSerialTask", 1024, NULL, 1, NULL);

}
// 연결 품질 모니터링
bool checkSignalQuality() {
    // 신호가 약할 때 처리
    if (WiFi.RSSI() < -70) {
        DEBUG_PORT.print("Signal strength (RSSI): ");
        DEBUG_PORT.print(WiFi.RSSI());
        DEBUG_PORT.println(" dBm");
        return false;
    }
    return true;
}

void led_on(int duration) {
    static unsigned long last_led_on_time = 0;
    static bool led_state = false;

    if( millis() - last_led_on_time > duration ) {
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
        last_led_on_time = millis();
   }
    else {
        digitalWrite(LED_PIN, led_state);
    }
}

void loop() {
    static bool led_state = false;
    int len = 0;
    
    bool signal_quality = checkSignalQuality();
    
    do {
        sendDataToWifi();

        if( signal_quality == false ) {
            break;
        }

        led_on(500);
    }while(false);
}
