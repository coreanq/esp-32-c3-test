/*
  WiFiAccessPoint.ino creates a WiFi access point and provides a web server on it.

  Steps:
  1. Connect to the access point "yourAp"
  2. Point your web browser to http://192.168.4.1/H to turn the LED on or http://192.168.4.1/L to turn it off
     OR
     Run raw TCP "GET /H" and "GET /L" on PuTTY terminal with 192.168.4.1 as IP address and 80 as port

  Created for arduino-esp32 on 04 July, 2018
  by Elochukwu Ifediora (fedy0)
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

// Set these to your desired credentials.
const char *ssid = "yourAP";
const char *password = "yourPassword";



void printHexString(const char* str, uint32_t length);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Init Debug Serial
  Serial.begin(921600);
  Serial.println();
  Serial.println("Debug Serial Init");
  
 
  // Init Comm Serial gpio 20 rx, gpio 21 tx
  Serial1.begin(460800, SERIAL_8N1, 20, 21);
  Serial.println("Comm Serial Init");
  
  // Set gpio10 to output
  // 485 enable so set low to Receive mode
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  
}

char buffer[256];

void loop() {
  
  uint32_t bytesRead = Serial1.readBytes(buffer, 256);

  if( bytesRead > 0 ) {
    printHexString(buffer, bytesRead);
  }

}

void printHexString(const char* str, uint32_t length) {

  for(uint32_t i = 0; i < length; i++) {
    char buffer[3]; // 각 문자를 16진수로 변환하는 버퍼 (2자리 + null 문자)
    sprintf(buffer, "%02X", (unsigned char)*str);
    Serial.print(buffer);
    str++;
  }
  Serial.println(); // 줄 바꿈
}