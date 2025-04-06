#include <SoftwareSerial.h>
#include <Arduino.h>
#include "APC220.h"
#include "settings.h"
#include <Wire.h>

const int vrs = 101;
const int rxPin = 10; //rx pin is tx pin on apc
const int txPin = 11; //tx pin is rx pin on apc
const int setPin = 8; //set pin
const int enPin = 12; // no use
const int fiveV = 13;  // 5V to the APC220

SoftwareSerial apc220(rxPin, txPin);
//APC220 coms(PIN_APC_SET, APC220_UART_BAUDRATE_9600);



void setupSoftAPC(void){
  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);
  delay (10);
  pinMode(fiveV, OUTPUT);  // 5V
  digitalWrite(fiveV, HIGH); // turn on 5V
  delay(50);
  pinMode(enPin, OUTPUT); // ENABLE
  digitalWrite(enPin, HIGH); //
  delay(100);
  apc220.begin(9600);
}

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("Hello from ground station MCU!");
   Serial.println(__DATE__);
  setupSoftAPC();
  delay(3000);
  Serial.println(F(" HDOP  Latitude   Longitude   ALT   Date       Time     Date  pressure  temerature  "));
  Serial.println(F("           (deg)      (deg)   (m)                       Age    (pa)        (c)"));
  Serial.println(F("--------------------------------------------------------------------------------------------------"));
}

  
 

void loop() {
  delay(1000);
  while (apc220.available()>1) {
  
    Serial.println(apc220.read(), HEX);
    
  }
}
