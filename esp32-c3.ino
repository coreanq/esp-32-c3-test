#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ESP-NOW 슬레이브 정보를 저장할 전역 변수
esp_now_peer_info_t slave = {0,};

#define LED_PIN 8
#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 0
#define DEBUG_MSG_BUFFER_SIZE 4096

// 데이터 전송을 위한 구조체 정의
typedef struct struct_message {
    char message[DEBUG_MSG_BUFFER_SIZE];
} struct_message;

struct_message myData;
struct_message myRecvData;

QueueHandle_t msgQueue;
bool isTxDone = true;

// 함수 선언
void OnDataRecv(const esp_recv_info_t *esp_now_info, const uint8_t *data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void deletePeer();

// 데이터 전송 콜백 함수
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial1.print("Send data to : ");   
    Serial1.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial1.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// 데이터 수신 콜백 함수
void OnDataRecv(const esp_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    memcpy(&myRecvData, data, data_len);
    myRecvData.message[data_len] = '\0';
    Serial1.print("recv data : ");
    // Serial1.printf("%02X:%02X:%02X:%02X:%02X:%02X", esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2], esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5]);
    Serial1.println(myRecvData.message);
}

// ESP-NOW 초기화 함수
void InitESPNow() {
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        Serial1.println("ESPNow Init Success");
    } else {
        Serial1.println("ESPNow Init Failed");
        ESP.restart();
    }
}

// 슬레이브 장치 스캔 함수
void ScanForSlave() {
    int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);
    bool slaveFound = 0;
    memset(&slave, 0, sizeof(slave));

    Serial1.println("");
    if (scanResults == 0) {
        Serial1.println("No WiFi devices in AP Mode found");
    } else {
        Serial1.print("Found "); Serial1.print(scanResults); Serial1.println(" devices ");
        for (int i = 0; i < scanResults; ++i) {
            // 스캔된 장치의 정보 가져오기
            String SSID = WiFi.SSID(i);
            int32_t RSSI = WiFi.RSSI(i);
            String BSSIDstr = WiFi.BSSIDstr(i);

            if (PRINTSCANRESULTS) {
                Serial1.print(i + 1); Serial1.print(": ");
                Serial1.print(SSID); Serial1.print(" (");
                Serial1.print(RSSI); Serial1.print(")");
                Serial1.println("");
            }
            delay(10);

            // 'Slave'로 시작하는 SSID를 찾음
            if (SSID.indexOf("Slave") == 0) {
                Serial1.println("Found a Slave.");
                Serial1.print(i + 1); Serial1.print(": "); Serial1.print(SSID);
                Serial1.print(" ["); Serial1.print(BSSIDstr); Serial1.print("]");
                Serial1.print(" ("); Serial1.print(RSSI); Serial1.print(")");
                Serial1.println("");

                // MAC 주소 파싱
                int mac[6];
                if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",
                               &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
                    for (int ii = 0; ii < 6; ++ii) {
                        slave.peer_addr[ii] = (uint8_t) mac[ii];
                    }
                }

                slave.channel = CHANNEL;  // 채널 설정
                slave.encrypt = 0;        // 암호화 비활성화
                slaveFound = 1;
                break;
            }
        }
    }

    if (slaveFound) {
        Serial1.println("Slave Found, processing..");
    } else {
        Serial1.println("Slave Not Found, trying again.");
    }

    WiFi.scanDelete();  // 스캔 결과 정리
}

// 슬레이브 관리 함수
bool manageSlave() {
    if (slave.channel == CHANNEL) {
        if (DELETEBEFOREPAIR) {
            deletePeer();
        }

        // 페어링 상태 확인
        Serial1.print("Slave Status: ");
        bool exists = esp_now_is_peer_exist(slave.peer_addr);
        if (exists) {
            Serial1.println("Already Paired");
            return true;
        } else {
            // 새로운 페어링 시도
            esp_err_t addStatus = esp_now_add_peer(&slave);
            if (addStatus == ESP_OK) {
                Serial1.println("Pair success");
                return true;
            } else {
                Serial1.println("Pair failed");
                return false;
            }
        }
    }
    Serial1.println("No Slave found to process");
    return false;
}

// 페어링된 슬레이브 삭제
void deletePeer() {
    esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
    Serial1.print("Slave Delete Status: ");
    if (delStatus == ESP_OK) {
        Serial1.println("Success");
    } else {
        Serial1.println("Error");
    }
}

// 데이터 전송 함수
void sendData() {
    const uint8_t *peer_addr = slave.peer_addr;
    int ItemCount = 0;
    do {
        if( isTxDone == false) {
            break;
        }
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

        if( ItemCount > 0) {
            esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &myData, ItemCount);

            myData.message[ItemCount] = '\0';
            // Serial1.print(myData.message);
            // Serial1.println("");

            if (result == ESP_OK) {
            } 
            else {
                isTxDone = true;
                Serial.println("Failed");
            }
        }
        else {
            isTxDone = true;
        }
    }while(false);
}

void recvDataToQueue() {
    int len = Serial0.readBytes(myData.message, DEBUG_MSG_BUFFER_SIZE);
    myData.message[len] = '\0';
    
    for ( int i = 0; i < len; i++ ) {
        xQueueSend( msgQueue, &myData.message[i], 0 );
    }
}

void setup() {
    // 시리얼 통신 초기화, before bgein
    Serial0.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    Serial0.setTimeout(1);
    Serial1.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    Serial1.setTimeout(1);

    Serial.begin(921600);
    Serial0.begin(3000000, SERIAL_8N1, 21, 20);  // RX:20, TX:21 핀 사용
    Serial1.begin(3000000, SERIAL_8N1, 1, 0);  //  DEBUG RX:1, TX:0 핀 사용


    pinMode(LED_PIN, OUTPUT);
    delay(1000);
    
    // WiFi 모드 설정
    WiFi.mode(WIFI_STA);

    // ESP-NOW 초기화 및 콜백 등록
    InitESPNow();

    Serial1.print("Init");
    Serial1.print("Current Date: ");
    Serial1.print(__DATE__);
    Serial1.print("Current Time: ");
    Serial1.println(__TIME__);
    Serial1.println(WiFi.softAPmacAddress().c_str());
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Queue 생성
    msgQueue = xQueueCreate( DEBUG_MSG_BUFFER_SIZE, sizeof(char) );
}


void loop() {
    static bool led_state = false;
    
    if( slave.peer_addr[0] == 0x00
        && slave.peer_addr[1] == 0x00
        && slave.peer_addr[2] == 0x00
        && slave.peer_addr[3] == 0x00
        && slave.peer_addr[4] == 0x00
        && slave.peer_addr[5] == 0x00
    ) {
        // 슬레이브 검색
        ScanForSlave();

        // 페어링 상태 확인 및 데이터 전송
        bool isPaired = manageSlave();
        if( !isPaired ) {
            Serial1.println("Slave pair failed!");
        }
    }
    else {
        
        recvDataToQueue(); 
        sendData();

        if( led_state ) {
            digitalWrite(LED_PIN, HIGH);
        }
        else {
            digitalWrite(LED_PIN, LOW);
        }
        led_state = !led_state;
    }
}