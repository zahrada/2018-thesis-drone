#include <Adafruit_BMP280.h> // include library so you can use barometr BMP280
#include <SoftwareSerial.h> // include the SoftwareSerial library so you can use its functions
#include <TinyGPS++.h>
/*
   This sample sketch should be the first you try out when you are testing a TinyGPS++
   (TinyGPSPlus) installation.  In normal use, you feed TinyGPS++ objects characters from
   a serial NMEA GPS device, but this example uses static strings for simplicity.
*/
// A sample NMEA stream.
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// The TinyGPS++ object
TinyGPSPlus gps;
double lat, lng, prevLat, prevLng;
float pidPXCoor, pidIXCoor, pidDXCoor, pidPYCoor, pidIYCoor, pidDYCoor;
float prevX = 0;
float prevY = 0;

// drone flyies into define altitute bu bmp sensor
float altitude = 5; // meters

float pitch, roll, yaw, throttle;

// set pins for hc-06 bluetooth module
#define rxPin 11
#define txPin 10

// declarate and define values for reseving data from bluetooth
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

// Set barometr BPM280
Adafruit_BMP280 bmp; // I2C
float pGround, pAir, pTempature, pAltitude;

//timer
unsigned long time;
float elapsedTime, timePrev;

float prevErrorAltitude = 0;
float pidPAltitude, pidIAltitude, pidDAltitude;

bool landing = false;

void setup() {

  mySerial.begin(9600);

  Serial.begin(9600);
  /*
    //initialization of barometr bmp
    if (!bmp.begin()) {
    mySerial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
    }

    // Default settings from datasheet.
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.

    // Read pressure hPa on the ground for relative altitude in hPa
    delay(1000);
    pGround = bmp.readPressure() / 100;
  */
  /*
      //set digital pin for reading droneProtocol
    pinMode(rollPin, INPUT);
    pinMode(pitchPin, INPUT);
    pinMode(yawPin, INPUT);
    pinMode(throttlePin, INPUT);
    pinMode(kpPin, INPUT);
    pinMode(kiPin, INPUT);
    pinMode(kdPin, INPUT);
  */

  Serial.print("We can begin :] ");

  while (*gpsStream)
    if (gps.encode(*gpsStream++))
      if (gps.location.isValid())
      {
        lat = gps.location.lat();
        lng = gps.location.lng();
      }

  time = millis(); //Start counting time in milliseconds
}

void loop() {

  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;

  //Read barometr data - altitute from referent point = start point
  //pAltitude = bmp.readAltitude(pGround);

  if (Serial.available() > 0)
  {

    byte rc = Serial.read();
    Serial.print("throtle - ");
    Serial.println(throttle);

  }
}

void setAltitude(float altitude, float pAltitude, float &throttle, float elapsedTime) {

  //define PID koef
  float kp = 1; //present
  float ki = 0.1; //past
  float kd = 1; // future

  // calculate error between what we want and what we have
  float errorAltitude = pAltitude - altitude;

  // calculate linear, integral and derivate part od PID
  pidPAltitude = kp * errorAltitude;

  pidIAltitude += ki * errorAltitude;
  pidDAltitude = kd * ((errorAltitude - prevErrorAltitude) / elapsedTime);

  //sumary all parts od PID
  float pidAltitude = pidPAltitude + pidIAltitude + pidDAltitude;
  Serial.println(pidAltitude);

  throttle = minMax(throttle + pidAltitude, throttle, 2000);
  // need constant which repair additive to throtle

  prevErrorAltitude = errorAltitude;
}

void setCoordinates(double lat, double lng, double prevLat, double prevLng, float &pitch, float &roll, float elapsedTime) {

  //compute azimuth of oriantacion of drone above Earth
  float azimuth = atan2( ( lat - prevLat ) , ( lng - prevLng ) );

  //compute distance between what possition want and what have
  float dist = sqrt( ( lng - prevLng ) * ( lng - prevLng ) + ( lat - prevLat ) * ( lat - prevLat ) );

  //compute X and Y in inerSystem to PID roll and pich, X and Y is error for PID
  float x = dist * cos(azimuth) - dist * sin(azimuth);
  float y = dist * sin(azimuth) + dist * cos(azimuth);

  //define PID koef
  float kp = 1; //present
  float ki = 0.1; //past
  float kd = 1; // future

  pidPXCoor = kp * x;
  pidIXCoor += ki * x;
  pidDXCoor = kd * ((x - prevX) / elapsedTime);
  pidPYCoor = kp * y;
  pidIYCoor += ki * y;
  pidDYCoor = kd * ((y - prevY) / elapsedTime);

  float pidX = pidPXCoor + pidIXCoor + pidDXCoor;
  float pidY = pidPYCoor + pidIYCoor + pidDYCoor;

  roll = minMax(roll + pidX, roll, 25);
  pitch = minMax(pitch + pidY, pitch, 25);
  // need constant which repair additive to throtle

  prevX = x;
  prevY = y;

}

static  float minMax(float value, float min_value, float max_value) {
  // fuction which reduce value over interval <min_value; max_value>
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }
  return value;
}


static void readObstacleControl(HardwareSerial Serial, float &altitude, bool landing) {
  //read data from obstacle control and change navigation of drone
  // if Serial reads value 0-4 obstacle is near by drone

  /*
                   2
               /\     /\
               ||     ||
             dist1  dist2
                1    2

    1  < - dist6 6  (dist7)  3  dist3 ->  3
                   0
                5    4
             dist5  dist4
               ||     ||
               \/     \/
                   4
  */

  if (Serial.available() > 0)
  {
    byte rc = Serial.read();

    switch (rc) {
      case 0:
        if (!landing) {
          altitude += 1;
        }
        break;
      case 1:
        altitude += 1;
        break;
      case 2:
        altitude += 1;
        break;
      case 3:
        altitude += 1;
        break;
      case 4:
        altitude += 1;
        break;
      default:
        altitude += 1;
        break;
    }
  }
}

static void sendData2FlyCtrl(SoftwareSerial mySerial, float pitch, float roll, float yaw, int throttle) {
  // send data to flying conroler by UART
  // x - break the reading while loop
  
  mySerial.print("p");
  mySerial.print(pitch);
  mySerial.print("r");
  mySerial.print(roll);
  mySerial.print("y");
  mySerial.print(yaw);
  mySerial.print("t");
  mySerial.print(throttle);
  mySerial.print("x");
}
