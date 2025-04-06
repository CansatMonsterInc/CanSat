#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include "APC220.h"
#include "settings.h"
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <TinyGPSPlus.h>


Adafruit_BMP280 bmp;
TinyGPSPlus gps;

SoftwareSerial apcSerial(PIN_APC_RX, PIN_APC_TX);
SoftwareSerial gpsSerial(5, 6);

unsigned long lastBmp280DataSent = 0;
static const uint32_t GPSBaud = 9600;

bool isRunning = true; // Flag to track if the system is running
int altitude_counter = 0; // Counter to track the altitude passes



//Custom type to convert floats to bytes, the transmit function requires a uint8_t array
/*
typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t temperature;
FLOATUNION_t pressure;
FLOATUNION_t latitude;
FLOATUNION_t longitude;
FLOATUNION_t altitude;
*/




// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      apcSerial.print('*');
    apcSerial.print(' ');
  }
  else
  {
    apcSerial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      apcSerial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  apcSerial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    apcSerial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    apcSerial.print(sz);
  }
  
  if (!t.isValid())
  {
    apcSerial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    apcSerial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}


void setup() {
  Serial.begin(9600);
  delay(100);
  DEBUG_SERIAL.println("serial started");
  gpsSerial.begin(GPSBaud);
  delay (100);
  DEBUG_SERIAL.println("APCserial started");
  apcSerial.begin(9600);
  delay(100);
  DEBUG_SERIAL.println("GPSserial started");
  

  DEBUG_SERIAL.println(F("Booting..."));

  #ifdef DEBUG_SCAN_I2C_DEVICES
  byte error, address;
  int nDevices = 0;

  DEBUG_SERIAL.println("Scanning for i2c devices...");
  Wire.begin();

  for (address = 1; address < 127; address++) {    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
     if (error == 0) {
      DEBUG_SERIAL.print("I2C device found at address 0x");
      if (address < 16) {
        DEBUG_SERIAL.print("0");
      }
      DEBUG_SERIAL.print(address, HEX);
      DEBUG_SERIAL.println("  !");
 
      nDevices++;
    } else if (error == 4) {
      DEBUG_SERIAL.print("Unknown error at address 0x");
      if (address < 16) {
        DEBUG_SERIAL.print("0");
      }
      DEBUG_SERIAL.println(address, HEX);
    }    
  }

  if (nDevices == 0) {
    DEBUG_SERIAL.println("No I2C devices found\n");
  } else {
    DEBUG_SERIAL.println("Done scanning!\n");
  }
  #endif

  DEBUG_SERIAL.println(F("Starting i2c link with BMP280"));
  if (!bmp.begin(I2C_BMP280_ADDRESS)) {
    DEBUG_SERIAL.println(F("Could not find the BMP280 sensor!"));
 
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  

  DEBUG_SERIAL.println(F("Configuring valve"));
  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_VALVE, LOW);  // Valve starts closed
  delay(10);

  void configure(long freq, int power, APC220_RF_BAUDRATE rfBaudrate, APC220_PARITY parity);

  DEBUG_SERIAL.println(F("Setup complete"));
  //make the teplate for the data packet
  apcSerial.println(F("--------------------------------------------------------------------------------------------------"));
  apcSerial.println(F(" HDOP  Latitude   Longitude   ALT   Date       Time     Date  pressure  temerature  "));
  apcSerial.println(F("           (deg)      (deg)   (m)                       Age    (pa)        (c)"));
  apcSerial.println(F("--------------------------------------------------------------------------------------------------"));
}
  
  
  unsigned long lastGpsDataSent = 0;
void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastGpsDataSent >= GPS_INTERVAL) {
    lastGpsDataSent = currentTime;
    printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    printDateTime(gps.date, gps.time);
   
    
    apcSerial.println();
   
   }
    
  if (millis() > 5000 && gps.charsProcessed() < 10)
  apcSerial.println(F("No GPS data received: check wiring"));
  
  
  
// Read and send data from the BMP280 sensor
if (currentTime - lastBmp280DataSent >= BMP280_INTERVAL) {
   lastBmp280DataSent = currentTime;

    //temperature.number = bmp.readTemperature();
    //pressure.number = bmp.readPressure();
  
  apcSerial.print(F(" "));
  apcSerial.print(bmp.readAltitude(1017.00));
  apcSerial.println(" m");

  apcSerial.print(F("  "));
  apcSerial.print(bmp.readTemperature());
  apcSerial.println(" *C");

  apcSerial.print(F("Pressure = "));
  apcSerial.print(bmp.readPressure());
 apcSerial.println(" Pa");
    // just for testing
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("altitude = "));
  Serial.print(bmp.readAltitude(1017.00));
  Serial.println(" m");
  Serial.print(bmp.readAltitude(1021.00));

    
  }

 //valve
 float previousAltitude = 0;   // Opslag voor vorige hoogte
 float valveOpenAltitude = -1; // Opslaan van hoogte bij valve open (-1 betekent nog niet geopend)
 bool valveOpened = false;     // Houdt bij of de valve al is geopend
 const float ALTITUDE_THRESHOLD = 870;  // Hoogte waarop we checken
 const float ERROR_MARGIN = 2.0;        // Foutmarge om kleine GPS fluctuaties te negeren
 const unsigned long VALVE_OPEN_TIME = 500; // Valve opent 0.5 seconde


  // Controleer of GPS-data geldig is
   {
    float currentAltitude = bmp.readAltitude(1017.00);
    
    // Stap 1: Controleer of we daadwerkelijk dalen**
   if (!valveOpened && currentAltitude < previousAltitude - ERROR_MARGIN) {
     

     // Stap 2: Check of we boven de drempel zaten en nu dalen**
     if (previousAltitude > ALTITUDE_THRESHOLD) {
      
      

       // Stap 4: Open valve voor 0.5 seconde en sluit daarna**
       
       apcSerial.println("Valve OPEN!");
       digitalWrite(PIN_VALVE, HIGH);  // Valve openen
        delay(VALVE_OPEN_TIME);         // 0.5 seconde wachten
       digitalWrite(PIN_VALVE, LOW);   // Valve sluiten
    
       apcSerial.println("Valve GESLOTEN!");

        // **Stap 5: Sla de hoogte op waarop de valve open ging**
        valveOpenAltitude = currentAltitude;
        apcSerial.print(F("Valve open at altitude: "));
        apcSerial.print(valveOpenAltitude);
       apcSerial.println(" ");
        valveOpened = true;  // Zorg ervoor dat dit maar één keer gebeurt
     }
   }
  }
}
