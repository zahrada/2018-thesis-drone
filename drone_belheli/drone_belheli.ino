/*ESC calibration sketch; author: ELECTRONOOBS */
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h> // include the SoftwareSerial library so you can use its functions
#include "MPU9250.h" //include library with gyro/akce/mag
#include "MahonyAHRS.h" // include library to filter data of gyro/akce/mag to roll, pich, yaw
#include <Adafruit_BMP280.h> // include library so you can use barometr BMP280

// set pins for hc-06 bluetooth module
#define rxPin 11
#define txPin 10

// set max/min signal for brushless motors calibration
const int maxSignal = 2000;
const int minSignal = 1000;

// set pin for each brushless motor
const int esc1Pin = 7;
const int esc2Pin = 6;
const int esc3Pin = 5;
const int esc4Pin = 4;
const int esc5Pin = 3;
const int esc6Pin = 2;

// declarate and define values for reseving data from bluetooth
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);
char resCommand;
int resSpeed, resInfo;
bool resBool = false;

// declarete brushless motor
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
Servo esc5;
Servo esc6;

// class for filter data from gyro/akce/mag
Mahony filter;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68,
MPU9250 IMU(Wire, 0x68);
float roll, pitch, heading;

// Set barometr BPM280
Adafruit_BMP280 bmp; // I2C
float pGround, pAir, pTempature, pAltitude;

void setup() {
  //start bluetooth communication
  mySerial.begin(9600);

  //start communication with IMU
  Wire.begin();
  while (!Serial) {}
  // start communication with IMU
  if (!IMU.begin()) {
    mySerial.print("IMU initialization unsuccessful");
    while (1);
  }

  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  //Mahonny filter
  filter.begin(50);

  //initialization of barometr bmp
  if (!bmp.begin()) {
    mySerial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // Read pressure hPa on the ground for relative altitude in hPa
  delay(1000);
  pGround = bmp.readPressure() / 100;

  mySerial.println("Drone is ready");
}

void loop() {
  // read IMU data
  IMU.readSensor();
  // Mahonny Filtracion
  // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and measured ones. short name local variable for readability
  filter.update(IMU.getGyroX_rads() * (180 / M_PI), IMU.getGyroY_rads() * (180 / M_PI), IMU.getGyroZ_rads() * (180 / M_PI), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());

  // Compute roll, pitch, yaw
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  // Compute direction to North from magnetometr not used
  float angl_north = angle_north( IMU.getMagX_uT(), IMU.getMagY_uT(), 0.07);


  //Read barometr data - altitute from referent point = start point
  pAltitude = bmp.readAltitude(pGround);

  // define values for reading data from bluetooth
  boolean  newData = false;
  byte resData[32];
  byte numReceived = 0;

  // read all data from bluetooth
  getBlueData(mySerial, numReceived, resData, newData);

  // Process data from bluetooth to command drone
  if (newData == true) {
    resBool = true;
    // separeta bluetooth data for next using
    separateBlueData(numReceived, resData, resCommand, resSpeed, resInfo);
  }

  // finnaly this do something with movement of drone :)
  doCommandDrone(resCommand, resSpeed, resInfo);
}

void getBlueData(SoftwareSerial &mySerial, byte &numReceived, byte resData[], boolean &newData) {
  // Read Bluetooth Data from HC-05
  // start message value is '<' and end is '>'
  static boolean recvInProgress = false;
  byte startMarker = '<';
  byte endMarker = '>';
  //byte lineMarker = '\n';
  //byte enterMarker = '\r';
  byte rc;
  static byte ndx = 0;

  //Read data from bluetooth
  while (mySerial.available() > 0) {
    rc = mySerial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        resData[ndx] = rc;
        ndx++;
      }
      else {
        resData[ndx] = '\0'; // terminate the string
        recvInProgress == false;
        newData = true;
        numReceived = ndx;  // save the number for use when printing
        ndx = 0;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void separateBlueData(byte &numReceived, byte resData[], char &resCommand, int &resSpeed, int &resInfo) {
  //Inicialition for NULL value
  resCommand = 'Y';
  resSpeed = 0;
  resInfo = 0;

  // B - balace
  // F - flying in direction
  // R - rotate
  // speed 0-99
  // Info - B-speed change 1 - 99, F - azimuth 1 - 99, R - CW(1-49)/CCW(50-99)

  for (int i = 0; i < numReceived; i++) {
    //Type of command
    if (resData[i] == 'B' || resData[i] == 'F' || resData[i] == 'R' || resData[i] == 'C' || resData[i] == 'D')
    {
      resCommand =  resData[i];
    }
    // Read Speed from command S01-99
    if (resData[i] == 'S') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rSpeed[2] = {resData[i + 1], resData[i + 2]};
        resSpeed = atoi(rSpeed);
      }
      else {
        resSpeed = 0;
        //mySerial.println("Try again");
      }
    }
    // Read Info of Command 01-99
    if (resData[i] == 'A') {

      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        resInfo = atoi(rInfo);
      }
      else {
        //mySerial.println("Try again");
      }
    }
  }

  //Finnal conditions
  if (resInfo == 0) {
    resSpeed = 0;
    //mySerial.println("Try again");
  }
  if (resSpeed == 0) {
    resInfo = 0;
    //mySerial.println("Try again");
  }
}

void doCommandDrone(char resCommand, int resSpeed, int resInfo) {
  //Command drone

  // B - balace
  // F - flying in direction
  // R - rotate
  // speed 0-99
  // Info - B-speed change 1 - 99, F - azimuth 1 - 99, R - CW(1-49)/CCW(50-99)

  if (resCommand == 'B') {
    
      if (resBool) {
      mySerial.print("balance ");
      mySerial.print(resSpeed);
      mySerial.print(" info ");
      mySerial.println(resInfo);
      }
    
    float balanc_angle = atan2(pitch, roll) * 180 / PI + 180;
    balanceDrone((resSpeed * 10) + 1000, (resInfo * 10), balanc_angle);

    resBool = false;
  }
  else if (resCommand == 'F') {
    if (resBool) {
      mySerial.print("flying ");
      mySerial.print(resSpeed);
      mySerial.print(" info ");
      mySerial.println(resInfo);
      mySerial.print(" map ");
      mySerial.println(map(resInfo, 0, 99, 0, 360));
    }

    flyDrone(1000, (resSpeed * 10) , map(resInfo, 0, 99, 0, 360));

    resBool = false;
  }
  else if (resCommand == 'R') {

    if (resBool) {
      mySerial.print("rotate ");
      mySerial.print(resSpeed);
      mySerial.print(" info ");
      mySerial.println(resInfo);
    }

    if (heading > 10 || heading < 350) {
      rotateDrone(1000, (resSpeed * 10), map(resInfo, 0, 99, 0, 360) );
    }

    resBool = false;
  }
  else if (resCommand == 'C') {
    if (resBool) {
      mySerial.println("ESC calibration...");
      escCalib(esc1, esc1Pin, esc2, esc2Pin, esc3, esc3Pin, esc4, esc4Pin, esc5, esc5Pin, esc6, esc6Pin, maxSignal, minSignal);
      mySerial.println("The ESCs are calibrated. Lets start :)");
    }

    resBool = false;
  }
  else if (resCommand == 'D') {
    mySerial.print(roll);
    mySerial.print(" ");
    mySerial.print(pitch);
    mySerial.print(" ");
    mySerial.print(heading);
    mySerial.print(" ");
    mySerial.println(pAltitude);

    esc1.writeMicroseconds(minSignal);
    esc2.writeMicroseconds(minSignal);
    esc3.writeMicroseconds(minSignal);
    esc4.writeMicroseconds(minSignal);
    esc5.writeMicroseconds(minSignal);
    esc6.writeMicroseconds(minSignal);
  }
  else {
    if (resBool) {
      mySerial.println("Try again");
    }
    resBool = false;

  }
}

void escCalib(Servo esc1, int esc1Pin, Servo esc2, int esc2Pin, Servo esc3, int esc3Pin, Servo esc4, int esc4Pin, Servo esc5, int esc5Pin, Servo esc6, int esc6Pin, int maxSignal, int minSignal)
{
  //Esc calibration set pin, minSpeed and maxSpeed
  // this is nesseery, without calibration bruslesh motor dont work

  // set brushsless motor by pin
  esc1.attach(esc1Pin);
  delay(10);
  esc2.attach(esc2Pin);
  delay(10);
  esc3.attach(esc3Pin);
  delay(10);
  esc4.attach(esc4Pin);
  delay(10);
  esc5.attach(esc5Pin);
  delay(10);
  esc6.attach(esc6Pin);

  delay(2000);
  esc1.writeMicroseconds(maxSignal);
  delay(2000);
  esc1.writeMicroseconds(minSignal);
  delay(2000);

  esc2.writeMicroseconds(maxSignal);
  delay(2000);
  esc2.writeMicroseconds(minSignal);
  delay(2000);

  esc3.writeMicroseconds(maxSignal);
  delay(2000);
  esc3.writeMicroseconds(minSignal);
  delay(2000);

  esc4.writeMicroseconds(maxSignal);
  delay(2000);
  esc4.writeMicroseconds(minSignal);
  delay(2000);

  esc5.writeMicroseconds(maxSignal);
  delay(2000);
  esc5.writeMicroseconds(minSignal);
  delay(2000);

  esc6.writeMicroseconds(maxSignal);
  delay(2000);
  esc6.writeMicroseconds(minSignal);
  delay(2000);

  delay(5000);
}

static float angle_north( float mx, float my, float magDeclinRad)
{
  //Calculate angle to North from magnetometr mx, my in degrees
  //Fix by magnetic Declination of lacation
  float angle_north = atan2(mx, my);
  angle_north += magDeclinRad;

  // corection of angle
  if (angle_north < 0)
    angle_north += 2 * PI;
  if (angle_north > 2 * PI)
    angle_north -= 2 * PI;

  return angle_north * 180 / M_PI;
}

static void controlSpeedBetween2Motors(int &speed_first, int &speed_second, float angle, int speed_change)
{
  // Calculate how much part of speed to change movement
  speed_second = speed_change * (60 - angle) / 60;
  speed_first = speed_change * angle / 60;
}

static void balanceDrone(int speed, int speedChange, float balanc_angle)
{
  // Control drone balance by IMU for hexacopter
  // Calculate direction of tilt and regulate speed of bruslesh motors
  // IMU - roll, pitch
  // Drone - esc, speed
  // axes +y (+pitch) is motor 3
  // axes +x (+roll) is between motor 4 and 5
  // compute balance angle from roll and pitch
  // reduce balance angle between 2 motors
  // set increase speed on two motor to balance drone
  // global value esc1 -> esc6 by Servo.h
  // global value MIN_SIGNAL = 1000

  if (balanc_angle > 0 && balanc_angle <= 30 )
  {
    int speed_first;
    int speed_second;

    balanc_angle += 30;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed + speed_first);
    esc3.writeMicroseconds(speed + speed_second);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 330 && balanc_angle <= 360)
  {
    int speed_first;
    int speed_second;

    balanc_angle -= 330;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed + speed_first);
    esc3.writeMicroseconds(speed + speed_second);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 30 && balanc_angle <= 90)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 30;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed + speed_first);
    esc2.writeMicroseconds(speed + speed_second);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 90 && balanc_angle <= 150)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 90;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed + speed_second);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed + speed_first);
  }
  else if (balanc_angle > 150 && balanc_angle <= 210)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 150;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed + speed_first);
    esc6.writeMicroseconds(speed + speed_second);
  }
  else if (balanc_angle > 210 && balanc_angle <= 270)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 210;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed + speed_first);
    esc5.writeMicroseconds(speed + speed_second);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 270 && balanc_angle <= 330)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 270;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed + speed_first);
    esc4.writeMicroseconds(speed + speed_second);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed);
  }
}

static void rotateDrone(int speed, int speedChange, float angle)
{
  if (angle < 180) {
    esc1.writeMicroseconds(speed + speedChange);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed + speedChange);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed + speedChange);
    esc6.writeMicroseconds(speed);
  }
  else {
    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed + speedChange);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed + speedChange);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed + speedChange);
  }
}

static void flyDrone(int speed, int speedChange, float balanc_angle)
{
  // conrol flying is controlled by rising speed on some motor
  // on drone is red led strip and this red led is 0, by CW is 60, 120, 180, 240, 300 -> 0
  if (balanc_angle > 0 && balanc_angle <= 60 )
  {
    int speed_first;
    int speed_second;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed + speed_first);
    esc5.writeMicroseconds(speed + speed_second);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 60 && balanc_angle <= 120)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 60;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed + speed_first);
    esc6.writeMicroseconds(speed + speed_second);
  }
  else if (balanc_angle > 120 && balanc_angle <= 180)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 120;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed + speed_second);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed + speed_first);
  }
  else if (balanc_angle > 180 && balanc_angle <= 240)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 180;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed + speed_first);
    esc2.writeMicroseconds(speed + speed_second);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 240 && balanc_angle <= 300)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 240;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed + speed_first);
    esc3.writeMicroseconds(speed + speed_second);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 300 && balanc_angle <= 360)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 300;

    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    speed_first = speedChange;
    speed_second = speedChange;

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed + speed_first);
    esc4.writeMicroseconds(speed + speed_second);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed);
  }
}
