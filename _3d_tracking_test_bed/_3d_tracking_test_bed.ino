#include <SPI.h>
#include <SD.h>
const int chipSelect = 10;
File dataFile;

#include <EEPROM.h>
int address = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_GPS.h>

#define GPSECHO  false
#define LED_PIN 13
#define gpsSerial Serial3

Adafruit_GPS GPS(&gpsSerial);
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

boolean usingInterrupt = false;
void useInterrupt(boolean);

int fileNumber;
char fileName[] = "3DTrack0000.txt";

int LED = 12;

unsigned long previousMillis1 = 0;
const long interval1 = 100;
unsigned long previousMillis2 = 0;
const long interval2 = 1000;

String logData = "";

void setup() {
  logData.reserve(200);
  
  Serial.begin(115200);
  fileNumber = EEPROM.read(address) + 1;
  EEPROM.write(address, fileNumber);

  if(fileNumber>999){
    fileName[7] = '0'+fileNumber/1000;
  }
  if(fileNumber>99){
    int hund = 0;
    hund = (fileNumber%1000)/100;
    fileName[8] = '0'+hund;
  }
  if(fileNumber>9){
    int tens = 0;
    tens = (fileNumber%100)/10;
    fileName[9] = '0'+tens;
  }
  int units = 0;
  units = fileNumber%10;
  fileName[10] = '0'+units;
  
  Serial.println("FILE NAME:");
  Serial.println(fileName);

  GPS.begin(9600);
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  #ifdef __arm__
    usingInterrupt = false;
  # else
    useInterrupt(true);
  #endif
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  accelgyro.initialize();

  //pinMode(LED, OUTPUT);
  pinMode(chipSelect,OUTPUT);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while(1){
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(900);
    }
    return;
  }
  Serial.println("card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open(fileName, FILE_WRITE);  
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__


void loop(){
  
  if (!usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO)
          if (c) Serial.print(c);
  }
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return; 
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    logData = "IMU,"+String(ax)+","+String(ay)+","+String(az)+","+String(gx)+","+String(gy)+","+String(gz)+"\n";
    Serial.println(logData);
    dataFile.print(logData);
    dataFile.flush();
  }

  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;
    logData = "Time: "+ String(GPS.hour, DEC)+':'+String(GPS.minute, DEC)+':'+String(GPS.seconds, DEC)+'.'+String(GPS.milliseconds)+"Date: "+String(GPS.day, DEC)+'/'+String(GPS.month, DEC)+"/20"+String(GPS.year, DEC)+"Fix: "+String((int)GPS.fix)+" quality: "+String(((int)GPS.fixquality))+"\n";
    Serial.print(logData);
    dataFile.print(logData);
    if (GPS.fix) {
      pinMode(LED,HIGH);
      logData = "Location: "+String(GPS.latitude, 2)+String(GPS.lat)+", "+String(GPS.longitude, 2)+String(GPS.lon)+"\nSpeed (knots): "+String(GPS.speed)+"Angle: "+String(GPS.angle)+"Altitude: "+String(GPS.altitude)+"Satellites: "+String((int)GPS.satellites)+"\n";
      Serial.print(logData);
      dataFile.print(logData);
    }else{
      pinMode(LED,LOW);
    }
    dataFile.flush();
  }
}
