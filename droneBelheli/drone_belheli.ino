/*
  Flying control
  Using commands
  B - balance dorne
  F - flying in direction
  R - rotate drone
  E - try each motor one by one
  C - calibrate esc, do it first

  S - speed 0-99
  G - speed change 0-99
  A - info 0-99, angle, number of esc
  P - PID kp input/10
  I - PID ki input/100
  D - PID kd input/10

*/
#include <Servo.h>
#include <Wire.h>
#include "MPU9250.h" //include library with gyro/akce/mag
#include "MahonyAHRS.h" // include library to filter data of gyro/akce/mag to roll, pich, yaw
#include <SoftwareSerial.h> // include the SoftwareSerial library so you can use its functions

unsigned long time;

//PID
float kp, ki, kd;
float elapsedTime, timePrev;
float prevErrorRoll = 0;
float prevErrorPitch = 0;
float prevErrorYaw = 0;

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
int resSpeed, resInfo, resChange, resuSpeed, resuChange;
bool resBool = false;
int reSpeed = 0;
int reChange = 0;
const int maxChangeSpeed = 11;
const int maxChangeChange = 6;

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
float roll, pitch, yaw;

void setup() {
  //start bluetooth communication
  mySerial.begin(9600);
  Serial.begin(9600);

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

  mySerial.println("Drone is ready");
  time = millis(); //Start counting time in milliseconds
}

void loop() {

  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;

  // read IMU data

  IMU.readSensor();
  // Mahonny Filtracion
  // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and measured ones. short name local variable for readability
  filter.update(IMU.getGyroX_rads() * (180 / M_PI), IMU.getGyroY_rads() * (180 / M_PI), IMU.getGyroZ_rads() * (180 / M_PI), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());

  // Compute roll, pitch, yaw
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  /*
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(heading);
  */
  /*
    // Compute direction to North from magnetometr not used
    float angl_north = angle_north( IMU.getMagX_uT(), IMU.getMagY_uT(), 0.07);
  */
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
    separateBlueData(numReceived, resData, resCommand, resSpeed, resInfo, kp, ki, kd);
  }

  // finnaly this do something with movement of drone :)
  doCommandDrone(resCommand, resSpeed, resInfo, kp, ki, kd);
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

void separateBlueData(byte &numReceived, byte resData[], char &resCommand, int &resSpeed, int &resInfo, float &kp, float &ki, float &kd) {
  //Inicialition for NULL value
  resCommand = 'Y';
  resSpeed = 0;
  resInfo = 0;
  resChange = 0;


  // B - balace
  // F - flying in direction
  // R - rotate
  // speed 0-99
  // Info - B-speed change 1 - 99, F - azimuth 1 - 99, R - CW(1-49)/CCW(50-99)

  for (int i = 0; i < numReceived; i++) {
    //Type of command
    if (resData[i] == 'B' || resData[i] == 'F' || resData[i] == 'R' || resData[i] == 'C' || resData[i] == 'E' || resData[i] == 'X')
    {
      resCommand =  resData[i];
    }
    // Read Speed from command S01-99
    if (resData[i] == 'S') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rSpeed[2] = {resData[i + 1], resData[i + 2]};
        resuSpeed = atoi(rSpeed);

        if ( (resuSpeed - reSpeed) < maxChangeSpeed && resuSpeed < 100) {
          resSpeed = resuSpeed;
          reSpeed = resuSpeed;
        }
        else {
          resSpeed = reSpeed;
        }
      }
      else {
        resSpeed = reSpeed;
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
    // Read Info of Command 01-99
    if (resData[i] == 'G') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rChange[2] = {resData[i + 1], resData[i + 2]};
        resuChange = atoi(rChange);

        if ( (resuChange - reChange) < maxChangeChange && resuChange < 100) {
          resChange = resuChange;
          reChange = resuChange;
        }
        else {
          resChange = reChange;
        }
      }
      else {
        resChange = reChange;
      }
    }

    // Value to PID control system drone
    if (resData[i] == 'P') {

      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        kp = atof(rInfo) / 10;
      }
      else {
        //mySerial.println("Try again");
      }
    }
    if (resData[i] == 'I') {

      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        ki = atof(rInfo) / 100;
      }
      else {
        //mySerial.println("Try again");
      }
    }
    if (resData[i] == 'D') {

      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        kd = atof(rInfo) / 1000;
      }
      else {
        //mySerial.println("Try again");
      }
    }
  }

  //Finnal conditions
  if (resChange == 0) {
    resSpeed = 0;
    //mySerial.println("Try again");
  }
  if (resSpeed == 0) {
    resChange = 0;
    //mySerial.println("Try again");
  }
}


void doCommandDrone(char resCommand, int resSpeed, int resInfo, float kp, float ki, float kd) {
  //Command drone
  if (resCommand == 'B') {

    if (resBool) {
      mySerial.print("kp ");
      mySerial.print(kp);
      mySerial.print(" ki ");
      mySerial.print(ki);
      mySerial.print(" kd ");
      mySerial.println(kd);
    }

    //balanceDrone(pitch, roll, (resSpeed * 10) + 1000, (resChange * 10));
    balancePID(pitch, roll, yaw, 0, 0, yaw, kp, ki, kd, (resSpeed * 10) + 1000);

    resBool = false;
  }
  else if (resCommand == 'F') {
    if (resBool) {
      /*
        mySerial.print("flying ");
        mySerial.print(resSpeed);
        mySerial.print(" info ");
        mySerial.print(resInfo);
        mySerial.print(" change ");
        mySerial.println(resChange);
        mySerial.print(" map ");
        mySerial.println(map(resInfo, 0, 99, 0, 360));
      */
    }
    // set pitch
    resInfo = map(resInfo, 0, 99, -33, 33);
    //set roll
    resChange = map(resChange, 0, 99, -33, 33);

    //flyDrone(1000 + (resSpeed * 10) , (resChange * 10), map(resInfo, 0, 99, 0, 360));
    balancePID(pitch, roll, yaw, resInfo, resChange, yaw, kp, ki, kd, (resSpeed * 10) + 1000);

    resBool = false;
  }
  else if (resCommand == 'R') {

    if (resBool) {
      mySerial.print("rotate ");
      mySerial.print(resSpeed);
      mySerial.print(" info ");
      mySerial.print(resInfo);
      mySerial.print(" change ");
      mySerial.println(resChange);
    }
    
    // set pitch
    resInfo = map(resInfo, 0, 99, -33, 33);
    balancePID(pitch, roll, yaw, 0, 0, resInfo, kp, ki, kd, (resSpeed * 10) + 1000);
    //rotateDrone(1000 + (resSpeed * 10) , (resChange * 10), map(resInfo, 0, 99, 0, 360) );


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
  else if (resCommand == 'X') {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    esc5.writeMicroseconds(1000);
    esc6.writeMicroseconds(1000);
  }
  else if (resCommand == 'E') {
    if (resBool) {
      mySerial.print("esc ");
      mySerial.print(resSpeed);
      mySerial.print(" number ");
      mySerial.println(resInfo);
    }
    if (resInfo == 1) {
      esc1.writeMicroseconds(resSpeed * 10 + 1000);
      esc2.writeMicroseconds(minSignal);
      esc3.writeMicroseconds(minSignal);
      esc4.writeMicroseconds(minSignal);
      esc5.writeMicroseconds(minSignal);
      esc6.writeMicroseconds(minSignal);
    }
    else if (resInfo == 2) {
      esc1.writeMicroseconds(minSignal);
      esc2.writeMicroseconds(resSpeed * 10 + 1000);
      esc3.writeMicroseconds(minSignal);
      esc4.writeMicroseconds(minSignal);
      esc5.writeMicroseconds(minSignal);
      esc6.writeMicroseconds(minSignal);
    }
    else if (resInfo == 3) {
      esc1.writeMicroseconds(minSignal);
      esc2.writeMicroseconds(minSignal);
      esc3.writeMicroseconds(resSpeed * 10 + 1000);
      esc4.writeMicroseconds(minSignal);
      esc5.writeMicroseconds(minSignal);
      esc6.writeMicroseconds(minSignal);
    }
    else if (resInfo == 4) {
      esc1.writeMicroseconds(minSignal);
      esc2.writeMicroseconds(minSignal);
      esc3.writeMicroseconds(minSignal);
      esc4.writeMicroseconds(resSpeed * 10 + 1000);
      esc5.writeMicroseconds(minSignal);
      esc6.writeMicroseconds(minSignal);
    }
    else if (resInfo == 5) {
      esc1.writeMicroseconds(minSignal);
      esc2.writeMicroseconds(minSignal);
      esc3.writeMicroseconds(minSignal);
      esc4.writeMicroseconds(minSignal);
      esc5.writeMicroseconds(resSpeed * 10 + 1000);
      esc6.writeMicroseconds(minSignal);
    }
    else if (resInfo == 6) {
      esc1.writeMicroseconds(minSignal);
      esc2.writeMicroseconds(minSignal);
      esc3.writeMicroseconds(minSignal);
      esc4.writeMicroseconds(minSignal);
      esc5.writeMicroseconds(minSignal);
      esc6.writeMicroseconds(resSpeed * 10 + 1000);
    }
    else {

    }

    resBool = false;

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
  esc1.writeMicroseconds(maxSignal);
  delay(1500);
  esc1.writeMicroseconds(minSignal);
  delay(1500);

  esc2.attach(esc2Pin);
  delay(10);
  esc2.writeMicroseconds(maxSignal);
  delay(1500);
  esc2.writeMicroseconds(minSignal);
  delay(1500);

  esc3.attach(esc3Pin);
  delay(10);
  esc3.writeMicroseconds(maxSignal);
  delay(1500);
  esc3.writeMicroseconds(minSignal);
  delay(1500);

  esc4.attach(esc4Pin);
  delay(10);
  esc4.writeMicroseconds(maxSignal);
  delay(1500);
  esc4.writeMicroseconds(minSignal);
  delay(1500);

  esc5.attach(esc5Pin);
  delay(10);
  esc5.writeMicroseconds(maxSignal);
  delay(1500);
  esc5.writeMicroseconds(minSignal);
  delay(1500);


  esc6.attach(esc6Pin);
  delay(10);
  esc6.writeMicroseconds(maxSignal);
  delay(1500);
  esc6.writeMicroseconds(minSignal);
  delay(1500);

}

static void balancePID(float pitch, float roll, float yaw, float desiredPitch, float desiredRoll, float desiredYaw, float kp, float ki, float kd, int throttle) {
  /*///////////////////////////P I D///////////////////////////////////*/
  /*Remember that for the balance we will use just one axis. I've choose the x angle
    to implement the PID with. That means that the x axis of the IMU has to be paralel to
    the balance*/

  /*First calculate the error between the desired angle and
    the real measured angle*/
  float errorPitch =  pitch - desiredPitch;
  float errorRoll = - roll + desiredRoll;
  float errorYaw = yaw - desiredYaw;

  /*Next the proportional value of the PID is just a proportional constant
    multiplied by the error*/

  float pidPpitch = kp * errorPitch;
  float pidProll = kp * errorRoll;
  float pidPYaw = kp * errorYaw;
  float pidIpitch = 0;
  float pidIroll = 0;
  float pidIYaw = 0;
  float pidDpitch, pidDroll, pidDYaw;

  /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -3 and 3 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/
  if (-3 < errorPitch < 3)
  {
    pidIpitch = pidIpitch + (ki * errorPitch);
  }
  if (-3 < errorRoll < 3)
  {
    pidIroll = pidIroll + (ki * errorRoll);
  }
  pidIYaw = pidIYaw + (ki * errorYaw);

  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time.
    Finnaly we multiply the result by the derivate constant*/
  pidDpitch = kd * ((errorPitch - prevErrorPitch) / elapsedTime);
  pidDroll = kd * ((errorRoll - prevErrorRoll) / elapsedTime);
  pidDYaw = kd * ((errorYaw - prevErrorYaw) / elapsedTime);

  float pidPitch = pidPpitch + pidIpitch + pidDpitch;
  float pidRoll = pidProll + pidIroll + pidDroll;
  float pidYaw = pidPYaw + pidIYaw + pidDYaw;

  /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/

  esc1.writeMicroseconds(minMax(throttle - pidPitch + pidYaw, throttle, 2000));
  esc2.writeMicroseconds(minMax(throttle - pidPitch + pidRoll - pidYaw, throttle, 2000));
  esc3.writeMicroseconds(minMax(throttle + pidPitch + pidRoll + pidYaw, throttle, 2000));
  esc4.writeMicroseconds(minMax(throttle + pidPitch - pidYaw, throttle, 2000));
  esc5.writeMicroseconds(minMax(throttle + pidPitch - pidRoll + pidYaw, throttle, 2000));
  esc6.writeMicroseconds(minMax(throttle - pidPitch - pidRoll - pidYaw, throttle, 2000));


  //Remember to store the previous error.
  float prevErrorRoll = errorRoll;
  float prevErrorPitch = errorPitch;
  float prevErrorYaw = errorYaw;
}


static void balanceDrone(float pitch, float roll, int speed, int speedChange)
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

  float balanc_angle = atan2(pitch, roll) * 180 / PI + 180;

  if (abs(pitch) < 0.5 && abs(roll) < 0.5) {
    speedChange = 0;
  }
  else {
    //Serial.println(sqrt(pitch * pitch + roll * roll) / 90);
    speedChange *= sqrt(pitch * pitch + roll * roll) / 90;
  }

  //Serial.println(speedChange);
  if (balanc_angle > 0 && balanc_angle <= 30 )
  {
    int speed_first;
    int speed_second;

    balanc_angle += 30;

    speed_first = speedChange;
    speed_second = speedChange;
    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed + speed_first);
    esc3.writeMicroseconds(speed + speed_second);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed - speed_first);
    esc6.writeMicroseconds(speed - speed_second);
  }
  else if (balanc_angle > 330 && balanc_angle <= 360)
  {
    int speed_first;
    int speed_second;

    balanc_angle -= 330;

    speed_first = speedChange;
    speed_second = speedChange;
    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed + speed_first);
    esc3.writeMicroseconds(speed + speed_second);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed - speed_first);
    esc6.writeMicroseconds(speed - speed_second);
  }
  else if (balanc_angle > 30 && balanc_angle <= 90)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 30;

    speed_first = speedChange;
    speed_second = speedChange;
    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    esc1.writeMicroseconds(speed + speed_first);
    esc2.writeMicroseconds(speed + speed_second);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed - speed_first);
    esc5.writeMicroseconds(speed - speed_second);
    esc6.writeMicroseconds(speed);
  }
  else if (balanc_angle > 90 && balanc_angle <= 150)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 90;

    speed_first = speedChange;
    speed_second = speedChange;
    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    esc1.writeMicroseconds(speed + speed_second);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed - speed_first);
    esc4.writeMicroseconds(speed - speed_second);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed + speed_first);
  }
  else if (balanc_angle > 150 && balanc_angle <= 210)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 150;

    speed_first = speedChange;
    speed_second = speedChange;
    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed - speed_first);
    esc3.writeMicroseconds(speed - speed_second);
    esc4.writeMicroseconds(speed);
    esc5.writeMicroseconds(speed + speed_first);
    esc6.writeMicroseconds(speed + speed_second);
  }
  else if (balanc_angle > 210 && balanc_angle <= 270)
  {
    int speed_first;
    int speed_second;
    balanc_angle -= 210;

    speed_first = speedChange;
    speed_second = speedChange;
    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);

    esc1.writeMicroseconds(speed - speed_first);
    esc2.writeMicroseconds(speed - speed_second);
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

    speed_first = speedChange;
    speed_second = speedChange;
    //controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speedChange);,

    esc1.writeMicroseconds(speed - speed_second);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed + speed_first);
    esc4.writeMicroseconds(speed + speed_second);
    esc5.writeMicroseconds(speed);
    esc6.writeMicroseconds(speed - speed_first);
  }

}

static void rotateDrone(int speed, int speedChange, float angle)
{
  // rotate dron to set angle
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

static  float minMax(float value, float min_value, float max_value) {
  // fuction which reduce value over interval <min_value; max_value>
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }
  return value;
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
