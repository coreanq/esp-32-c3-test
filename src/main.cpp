#define BLYNK_TEMPLATE_ID "TMPL6pCBFFrxM"
#define BLYNK_TEMPLATE_NAME "LED ESP32"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial

#define APP_DEBUG

#include "BlynkEdgent.h"

#define LED_PIN 8  // Use pin 2 for LED (change it, if your board uses another pin)


BLYNK_WRITE(V0)
{
  // Local variable `value` stores the incoming LED switch state (1 or 0)
  // Based on this value, the physical LED on the board will be on or off:
  int value = param.asInt();

  if (value == 1) {
    digitalWrite(LED_PIN, HIGH);
    Serial.print("value =");
    Serial.println(value);
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.print("value = ");
    Serial.println(value);
  }
}
void setup()
{
  // pinMode(LED_PIN, OUTPUT);

  // Debug console. Make sure you have the same baud rate selected in your serial monitor
  Serial.begin(921600);
  delay(100);
  #if 0 
  // AP 모드 설정을 위한 WiFi 초기화
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_AP);
  delay(2000);
  // AP 정보 출력
  Serial.print("AP Name: ");
  Serial.println(getWiFiName());
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  BlynkEdgent.begin();
  #endif
  BlynkEdgent.begin();

  const char* AP_SSID = "ESP32-TEST-AP";  // 알아보기 쉬운 이름으로 설정
  const int WIFI_CHANNEL = 1;

  // WiFi 완전 초기화
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  
  // AP 모드 설정
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // AP 시작
  bool success = WiFi.softAP(AP_SSID, "", WIFI_CHANNEL, 0);
  
  if(success) {
    Serial.println("AP Started Successfully");
    Serial.printf("AP Name: %s\n", AP_SSID);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("AP Channel: %d\n", WiFi.channel());
    Serial.printf("AP MAC: %s\n", WiFi.softAPmacAddress().c_str());
  } else {
    Serial.println("AP Start Failed!");
  }

}

void loop() {
  BlynkEdgent.run();
  // delay(10);
  // digitalWrite(8, HIGH);
  // delay(100);
  // digitalWrite(8, LOW);
  // delay(100);
}

