#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 13

String imuString = "";

#include <Adafruit_GPS.h>

#define mySerial Serial3

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

String gpsString = "";
String gpsString1 = "";

void setup()  
{
gpsString.reserve(500);
gpsString1.reserve(200);

  Serial.begin(115200);


  GPS.begin(9600);
  mySerial.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif


    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    accelgyro.initialize();

    // configure Arduino LED for
    imuString.reserve(200);
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

uint32_t timer = millis();
void loop()                     // run over and over again
{
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  if (GPS.newNMEAreceived()) {
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    gpsString="\nTime: "+ String(GPS.hour, DEC)+':'+String(GPS.minute, DEC)+':'+String(GPS.seconds, DEC)+'.'+String(GPS.milliseconds)+"\nDate: "+String(GPS.day, DEC)+'/'+String(GPS.month, DEC)+"/20"+String(GPS.year, DEC)+"\nFix: "+String((int)GPS.fix)+" quality: "+String(((int)GPS.fixquality));

    if (GPS.fix) {
      gpsString1="\nLocation: "+String(GPS.latitude, 2)+String(GPS.lat)+", "+String(GPS.longitude, 2)+String(GPS.lon)+"\nSpeed (knots): "+String(GPS.speed)+"\nAngle: "+String(GPS.angle)+"\nAltitude: "+String(GPS.altitude)+"\nSatellites: "+String((int)GPS.satellites);
      gpsString=gpsString+gpsString1;

    }    
    Serial.println(gpsString);
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
imuString="\nIMU,"+String(ax)+","+String(ay)+","+String(az)+","+String(gx)+","+String(gy)+","+String(gz);
        // display tab-separated accel/gyro x/y/z values
        Serial.println(imuString);  
  }
}
