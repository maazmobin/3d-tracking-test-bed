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
int LED = 12;
int BTN = 11;

#include <SPI.h>
#include "SdFat.h"

String inputString2 = "";         // a string to hold incoming data
boolean stringComplete2 = false;  // whether the string is complete



unsigned long previousMillis = 0;
unsigned long previousMillisGPS = 0;

const uint8_t chipSelect = 10;

const uint32_t SAMPLE_INTERVAL_MS = 20;
const uint32_t SAMPLE_INTERVAL_MS_GPS = 1000;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"
//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;

// Write data header.
void writeHeader() {
  file.print(F("millis,"));
  file.print(F("ax,ay,az,gx,gy,gz,"));
  file.print(F("hour,minute,second,day,month,year,"));
  file.print(F("fix,quality,"));
  file.print(F("lat,long,speed,angle,altitude,satellite"));
  //  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
  //    file.print(F(",adc"));
  //    file.print(i, DEC);
  //  }
  file.println();
}
//------------------------------------------------------------------------------
// Log a data record.
void logData(int gps) {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Write data to file.  Start with log time in micros.
  file.print(millis());
  file.write(',');
  file.print(ax);
  file.write(',');
  file.print(ay);
  file.write(',');
  file.print(az);
  file.write(',');
  file.print(gx);
  file.write(',');
  file.print(gy);
  file.write(',');
  file.print(gz);
  file.write(',');
  int odo=0;
  if (stringComplete2) {
    file.print(inputString2);
    // clear the string:
    inputString2 = "";
    stringComplete2 = false;
    odo=1;
  }
  if(gps==1){
    if(odo==0){
      file.print(",,,,,");
    }
    file.write(',');
    file.print(GPS.hour);
    file.write(',');
    file.print(GPS.minute);
    file.write(',');
    file.print(GPS.seconds);
    file.write(',');
    file.print(GPS.day);
    file.write(',');
    file.print(GPS.month);
    file.write(',');
    file.print(GPS.year);
    file.write(',');
    file.print((int)GPS.fix);
    file.write(',');
    file.print((int)GPS.fixquality);
    file.write(',');
//    Serial.print("LAT: ");
//    Serial.print(GPS.latitude, 8);
//    Serial.println(GPS.lat);
    file.print(GPS.latitude, 8);
    file.print(GPS.lat);
    file.write(',');
    file.print(GPS.longitude, 8);
    file.print(GPS.lon);
    file.write(',');
    file.print(GPS.speed, 4);
    file.write(',');
    file.print(GPS.angle, 4);
    file.write(',');
    file.print(GPS.altitude, 4);
    file.write(',');
    file.print(GPS.satellites);
  }
  file.println();
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
//------------------------------------------------------------------------------
void setup() {
  Serial2.begin(115200);
  inputString2.reserve(40);

  pinMode(LED, OUTPUT);
  pinMode(BTN, INPUT);
  digitalWrite(BTN, HIGH); // pulled Up
  
  GPS.begin(9600);
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(2000);

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
  
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";

  Serial.begin(115200);
  
  // Wait for USB Serial 
  while (!Serial) {
    SysCall::yield();
  }
  delay(1000);

  Serial.println(F("Type any character to start"));
  while (!Serial.available() && digitalRead(BTN)) {
    SysCall::yield();
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(1000);
  }

  while(digitalRead(BTN) == LOW){
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);    
    delay(1000);
  }
  
  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
    error("file.open");
  }
  // Read any Serial data.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  Serial.println(F("Type any character to stop"));

  // Write data header.
  writeHeader();

  // Start on a multiple of the sample interval.
  logTime = micros()/(1000UL*SAMPLE_INTERVAL_MS) + 1;
  logTime *= 1000UL*SAMPLE_INTERVAL_MS;
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






void loop() {
  if (!usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO)
          if (c) Serial.print(c);
  }
  if (GPS.newNMEAreceived()) {
  //    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return; 
  }

  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= SAMPLE_INTERVAL_MS) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    int gps = 0;
    if(currentMillis - previousMillisGPS >= SAMPLE_INTERVAL_MS_GPS){
      previousMillisGPS = currentMillis;
      gps=1;
    }
    else{
      gps=0;
    }
    logData(gps);

    // Force data to SD and update the directory entry to avoid data loss.
    if (!file.sync() || file.getWriteError()) {
      error("write error");
    }
  }
  
  if (digitalRead(BTN)==LOW) {
    delay(500);
    if(digitalRead(BTN)==LOW){
      // Close file and stop.
      file.close();
      Serial.println(F("Done"));
      while(digitalRead(BTN) == LOW){
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);    
        delay(500);
      }
      SysCall::halt();
    }
  }
}

void serialEvent2(){
  while (Serial2.available()) {
    // get the new byte:
    char inChar = (char)Serial2.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete2 = true;
    }else{
      inputString2 += inChar;
    }
  }
}
