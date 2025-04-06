#include <Arduino.h>
#include "APC220.h"
#include <SoftwareSerial.h>
#include "settings.h"
const int setPin = 8;
SoftwareSerial apc220(PIN_APC_RX, PIN_APC_TX);
APC220::APC220(int setPin, APC220_UART_BAUDRATE baudrate) {
    _setPin = setPin;
    _baudrate = baudrate;

    pinMode(setPin, OUTPUT);
    digitalWrite(setPin, HIGH);

    apc220.begin(baudrate);
    delay(100);
}

 
    void APC220::configure(long freq, int power, APC220_RF_BAUDRATE rfBaudrate, APC220_PARITY parity) {
      //Go into config mode
    digitalWrite(_setPin, LOW);
    delay(100);
    
    //Write config (format from sample code file)
  
    apc220.print("WR");
    apc220.print(" ");

    apc220.print(freq);
    apc220.print(" ");
  
    apc220.print(rfBaudrate);
    apc220.print(" ");
  
    apc220.print(power);
    apc220.print(" ");
  
    apc220.print(_baudrate);// do not change baudrate, this could corrupt future communication
    apc220.print(" ");
  
    apc220.print(parity);
  
    apc220.write(0x0D);
    apc220.write(0x0A);
    delay(500);
 

    //Exit config mode
    digitalWrite(_setPin, HIGH);
    delay(100);
    }
