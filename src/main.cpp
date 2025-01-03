#define BLYNK_TEMPLATE_ID "TMPL6pCBFFrxM"
#define BLYNK_TEMPLATE_NAME "LED ESP32"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial

#define APP_DEBUG

#include "BlynkEdgent.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  
  BLYNK_PRINT.begin(115200);
  // Serial.setPins(21, 20);

  delay(100);
  BlynkEdgent.begin();
  pinMode(8, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  BLYNK_PRINT.println("Hello World");
  delay(1000);
  digitalWrite(8, HIGH);
  delay(1000);
  digitalWrite(8, LOW);
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}