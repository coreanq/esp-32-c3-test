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

#define BYPASS_SRC_PORT Serial0
#define DEBUG_PORT      Serial1

// 데이터 전송을 위한 구조체 정의
typedef struct struct_message {
    char message[DEBUG_MSG_BUFFER_SIZE];
} struct_message;

struct_message myData;
struct_message myRecvData;

QueueHandle_t msgQueue;
bool isTxDone = true;

// 함수 선언
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void deletePeer();

// 데이터 전송 콜백 함수
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    DEBUG_PORT.print("Send data to : ");   
    DEBUG_PORT.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    DEBUG_PORT.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// 데이터 수신 콜백 함수
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    memcpy(&myRecvData, data, data_len);
    myRecvData.message[data_len] = '\0';
    DEBUG_PORT.print("recv data : ");
    // Serial1.printf("%02X:%02X:%02X:%02X:%02X:%02X", esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2], esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5]);
    DEBUG_PORT.println(myRecvData.message);
}

// ESP-NOW 초기화 함수
void InitESPNow() {
    if (esp_now_init() == ESP_OK) {
        DEBUG_PORT.println("ESPNow Init Success");
    } else {
        DEBUG_PORT.println("ESPNow Init Failed");
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
        DEBUG_PORT.println("No WiFi devices in AP Mode found");
    } else {
        DEBUG_PORT.print("Found "); DEBUG_PORT.print(scanResults); DEBUG_PORT.println(" devices ");
        for (int i = 0; i < scanResults; ++i) {
            // 스캔된 장치의 정보 가져오기
            String SSID = WiFi.SSID(i);
            int32_t RSSI = WiFi.RSSI(i);
            String BSSIDstr = WiFi.BSSIDstr(i);

            if (PRINTSCANRESULTS) {
                DEBUG_PORT.print(i + 1); DEBUG_PORT.print(": ");
                DEBUG_PORT.print(SSID); DEBUG_PORT.print(" (");
                DEBUG_PORT.print(RSSI); DEBUG_PORT.print(")");
                DEBUG_PORT.println("");
            }
            delay(10);

            // 'Slave'로 시작하는 SSID를 찾음
            if (SSID.indexOf("Slave") == 0) {
                DEBUG_PORT.println("Found a Slave.");
                DEBUG_PORT.print(i + 1); DEBUG_PORT.print(": "); DEBUG_PORT.print(SSID);
                DEBUG_PORT.print(" ["); DEBUG_PORT.print(BSSIDstr); DEBUG_PORT.print("]");
                DEBUG_PORT.print(" ("); DEBUG_PORT.print(RSSI); DEBUG_PORT.print(")");
                DEBUG_PORT.println("");

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
        DEBUG_PORT.println("Slave Found, processing..");
    } else {
        DEBUG_PORT.println("Slave Not Found, trying again.");
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
        DEBUG_PORT.print("Slave Status: ");
        bool exists = esp_now_is_peer_exist(slave.peer_addr);
        if (exists) {
            DEBUG_PORT.println("Already Paired");
            return true;
        } else {
            // 새로운 페어링 시도
            esp_err_t addStatus = esp_now_add_peer(&slave);
            if (addStatus == ESP_OK) {
                DEBUG_PORT.println("Pair success");
                return true;
            } else {
                DEBUG_PORT.println("Pair failed");
                return false;
            }
        }
    }
    DEBUG_PORT.println("No Slave found to process");
    return false;
}

// 페어링된 슬레이브 삭제
void deletePeer() {
    esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
    DEBUG_PORT.print("Slave Delete Status: ");
    if (delStatus == ESP_OK) {
        DEBUG_PORT.println("Success");
    } else {
        DEBUG_PORT.println("Error");
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
                DEBUG_PORT.println("Failed");
            }
        }
        else {
            isTxDone = true;
        }
    }while(false);
}

void recvDataToQueue() {
    int len = BYPASS_SRC_PORT.readBytes(myData.message, DEBUG_MSG_BUFFER_SIZE);
    myData.message[len] = '\0';
    
    for ( int i = 0; i < len; i++ ) {
        xQueueSend( msgQueue, &myData.message[i], 0 );
    }
}

void setup() {
    // 시리얼 통신 초기화, before bgein
    BYPASS_SRC_PORT.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    BYPASS_SRC_PORT.setTimeout(1);
    DEBUG_PORT.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    DEBUG_PORT.setTimeout(1);

    Serial.begin(921600);
    BYPASS_SRC_PORT.begin(3000000, SERIAL_8N1, 21, 20);  // RX:20, TX:21 핀 사용
    DEBUG_PORT.begin(3000000, SERIAL_8N1, 1, 0);  //  DEBUG RX:1, TX:0 핀 사용
    delay(1000);

    DEBUG_PORT.print("\n\n-------------------------------------------------------------------\n");
    DEBUG_PORT.print("Current Date: ");
    DEBUG_PORT.print(__DATE__);
    DEBUG_PORT.print(" ");
    DEBUG_PORT.println(__TIME__);

    pinMode(LED_PIN, OUTPUT);
    
    // WiFi 모드 설정
    WiFi.mode(WIFI_STA);

    DEBUG_PORT.print("STA MAC Address: ");
    DEBUG_PORT.println(WiFi.macAddress().c_str());

    // ESP-NOW 초기화 및 콜백 등록
    InitESPNow();
    
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
            DEBUG_PORT.println("Slave pair failed!");
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