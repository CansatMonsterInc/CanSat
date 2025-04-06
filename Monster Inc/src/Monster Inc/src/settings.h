#ifndef settings
#define settings_h

//Debug serial output
#define DEBUG_SERIAL            Serial
#define DEBUG_SERIAL_BAUDRATE   9600
//#define DEBUG_WAIT_FOR_SERIAL           //Comment this line out to disable waiting for debug serial link
#define DEBUG_SCAN_I2C_DEVICES          //Comment this line out to disable scanning for i2c devices

    
//COMMS settings
#define APC_FREQUENCY           433800  //TODO: replace this with the frequency assigned at launchday
#define APC_POWER               9       //0 .. 9 (9 = 20mW)
    

//gps pins
#define PIN_GPS_RX               5   
#define PIN_GPS_TX               6

//apc220 pins
//#define PIN_APC_SET             
#define PIN_APC_TX              3
#define PIN_APC_RX              8

//I2C pins/mbp pins 
#define I2C_SDA                 A4
#define I2C_SCL                 A5
#define I2C_BMP280_ADDRESS      0x76
//valve pins
#define PIN_VALVE                9

//Intervals etc...
#define GPS_INTERVAL            1000    //How often we should emit a heartbeat packet
#define BMP280_INTERVAL         1000    //How often we should transmit the temperature/pressure info



#endif //settings.h