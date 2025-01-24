#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define LED_PIN 8


#define CHANNEL 1
#define PRINTSCANRESULTS 3
#define DELETEBEFOREPAIR 0
#define DEBUG_MSG_BUFFER_SIZE 4096

#define DEBUG_PORT Serial

// 데이터 전송을 위한 구조체 정의
typedef struct struct_message {
    char message[DEBUG_MSG_BUFFER_SIZE];
} struct_message;
struct_message myData;

bool isConnected = false;
QueueHandle_t msgQueue;

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
void sendData() {
    int ItemCount = 0;
    do {
        ItemCount = uxQueueMessagesWaiting( msgQueue );
            
        if( ItemCount == 0 ){
            break;
        }

        if( ItemCount > 200 ) {
            // esp now packet size 256
            ItemCount = 200;
        }

        for( int i = 0; i < ItemCount; i++ ) {
            xQueueReceive( msgQueue, &myData.message[i],  0 );
        }

        esp_err_t result = esp_now_send(slave.peer_addr, (uint8_t *) myData.message, ItemCount);
        
        if (result == ESP_OK) {
            myData.message[ItemCount] = '\0';
            // DEBUG_PORT.print(myData.message);
        } else {
            DEBUG_PORT.println("Failed");
            DEBUG_PORT.print( "len : " );
            DEBUG_PORT.println( ItemCount );
            DEBUG_PORT.print( esp_err_to_name(result) );
            DEBUG_PORT.println("");
        }

    }while(false);
}        
        

void setup() {
    // 시리얼 통신 초기화, before bgein
    DEBUG_PORT.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    DEBUG_PORT.setTimeout(1);

    Serial.begin(921600);
    Serial0.begin(3000000, SERIAL_8N1, 1, 0);  //  DEBUG RX:1, TX:0 핀 사용

    delay(1000);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    DEBUG_PORT.print("\n\n-------------------------------------------------------------------\n");
    DEBUG_PORT.print("Date : ");
    DEBUG_PORT.println(__DATE__);
    DEBUG_PORT.print("Time : ");
    DEBUG_PORT.println(__TIME__);
    
    // WiFi 모드 설정
    WiFi.mode(WIFI_AP);
    configDeviceAP();

    // This is the mac address of the Slave in AP Mode
    DEBUG_PORT.print("AP MAC Address: "); 
    DEBUG_PORT.println(WiFi.softAPmacAddress());

    // Init ESPNow with a fallback logic
    InitESPNow();

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Queue 생성
    msgQueue = xQueueCreate( DEBUG_MSG_BUFFER_SIZE, sizeof(char) );
    
}

void loop() {
    static bool led_state = false;
    char buffer[DEBUG_MSG_BUFFER_SIZE];
    int len = 0;
    
    len = DEBUG_PORT.readBytes(buffer, DEBUG_MSG_BUFFER_SIZE);
    
    if( len > 0 ) {
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;

        for ( int i = 0; i < len; i++ ) {
            xQueueSend( msgQueue, buffer, 0 );
        }

        if( isConnected == true ) {
            sendData();
        }
        else{
            DEBUG_PORT.println("Not Connected, queue reset");
            if( xQueueReset( msgQueue ) != pdPASS) {
              DEBUG_PORT.println("Queue Reset Failed");
            }
        }
    }

}
