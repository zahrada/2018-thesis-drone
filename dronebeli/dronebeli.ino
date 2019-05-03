#include <Servo.h>
#include <Wire.h>

// declaration of time in millisecond
unsigned long time;
float elapsedTime, timePrev;

//stabilization PID
float kp, ki, kd;
float prevErrorRoll = 0;
float prevErrorPitch = 0;
float prevErrorYaw = 0;
float pidPitch, pidRoll, pidYaw, pidIpitch, pidIroll, pidIYaw;

//rate PID
float dkp, dki, dkd;
float dPrevErrorRoll = 0;
float dPrevErrorPitch = 0;
float dPrevErrorYaw = 0;
float dPidPitch, dPidRoll, dPidYaw, dPidIpitch, dPidIroll, dPidIYaw;

// set max/min Throttle for ESC and brushless motors calibration
const int maxSignal = 2000;
const int minSignal = 1000;

// set pin for each brushless motor
DDRD |= B11111100;

const int esc1Pin = 7;
const int esc2Pin = 6;
const int esc3Pin = 5;
const int esc4Pin = 4;
const int esc5Pin = 3;
const int esc6Pin = 2;

bool resBool = false;

float desiredPitch = 0;
float desiredRoll = 0;
float desiredYaw = 0;
int throttle;

// define values for reading data from bluetooth
boolean  newData = false;
byte resData[32];
byte numReceived = 0;

// declarate brushless motor
Servo esc1, esc2, esc3, esc4, esc5, esc6;

// declarate MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68,
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float pitch, roll, yaw, rollG;

//fuse do calibration before start flying with drone
bool firstCalib = false;
bool runDrone = true;

void setup() {

  //start Serial communication
  Serial.begin(250000);
  Serial.println("Start");

  Wire.begin(); //begin the wire comunication
  TWBR = 12;   //Set the I2C clock speed to 400kHz.
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Drone is ready");

  time = millis();//Start counting time in milliseconds
}

void loop() {
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;


  // read raw data from IMU
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  aX = Wire.read() << 8 | Wire.read(); //each value needs two registres
  aY = Wire.read() << 8 | Wire.read();
  aZ = Wire.read() << 8 | Wire.read();
  aX /= 16384.0;
  aY /= 16384.0;
  aZ /= 16384.0;

  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true); //Just 4 registers

  gX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
  gY = Wire.read() << 8 | Wire.read();
  gZ = Wire.read() << 8 | Wire.read();

  gX /= 131.0;
  gY /= 131.0;
  gZ /= 131.0;


  /*
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
  */

  //Complementary filter
  float accX = atan2((aY) , sqrt(pow(aX, 2) + pow(aZ, 2))) * (180 / M_PI);
  float accY = atan2(-1 * (aX) , sqrt(pow(aY, 2) + pow(aZ, 2)))  * (180 / M_PI);

  pitch = 0.99 * (pitch + gX  * elapsedTime) + 0.01 * accX;
  roll = 0.99 * (roll + gY * elapsedTime) + 0.01 * accY;

  rollG = rollG + gY  * elapsedTime;

  //Serial.print(accX);
  //Serial.print(" ");
  //Serial.print(rollG);
  //Serial.print(" ");
  //Serial.print(accY);
  //Serial.print(" ");
  //Serial.print(gY);
  //Serial.print(" ");
  //Serial.print(roll);
  //Serial.print(" ");
  //Serial.println(pitch);
  //Serial.print(" ");
  //Serial.println(roll);
  //Serial.print(" ");
  //Serial.println(pitch);
  //Serial.print(" ");
  //Serial.println(yaw);

  // read all data from SoftwareSerial
  getBlueData(numReceived, resData, newData);

  // Process data from SoftwareSerial to command drone
  if (newData == true) {
    resBool = true;
    // separete SoftwareSerial data for next using
    separateBlueData(numReceived, resData, kp, ki, kd, desiredPitch, desiredRoll, desiredYaw, throttle);
  }

  if (firstCalib && runDrone) {

    // stabilise PID
    calculatePID(pitch, desiredPitch, elapsedTime, kp, ki, kd, pidIpitch, prevErrorPitch, pidPitch);
    calculatePID(roll, desiredRoll, elapsedTime, kp, ki, kd, pidIroll, prevErrorRoll, pidRoll);
    calculatePID(yaw, desiredYaw, elapsedTime, kp, ki, kd, pidIYaw, prevErrorYaw, pidYaw);

    //rate PID
    calculatePID(gX, pidPitch, elapsedTime, dkp, dki, dkd, dPidIpitch, dPrevErrorPitch, dPidPitch);
    calculatePID(gY, pidRoll, elapsedTime, dkp, dki, dkd, dPidIroll, dPrevErrorRoll, dPidRoll);
    calculatePID(gZ, pidYaw, elapsedTime, dkp, dki, dkd, dPidIYaw, dPrevErrorYaw, dPidYaw);
/*
    Serial.print("1- ");
    Serial.print(minMax(throttle - dPidPitch + dPidYaw, minSignal, 1700));
    Serial.print(" 2- ");
    Serial.print(minMax(throttle - dPidPitch + dPidRoll - dPidYaw, minSignal, 1700));
    Serial.print(" 3- ");
    Serial.print(minMax(throttle + dPidPitch + dPidRoll + dPidYaw, minSignal, 1700));
    Serial.print(" 4- ");
    Serial.print(minMax(throttle + dPidPitch - dPidYaw, minSignal, 1700));
    Serial.print(" 5- ");
    Serial.print(minMax(throttle + dPidPitch - dPidRoll + dPidYaw, minSignal, 1700));
    Serial.print(" 6- ");
    Serial.println(minMax(throttle - dPidPitch - dPidRoll - dPidYaw, minSignal, 1700));
*/
    esc1.writeMicroseconds(minMax(throttle - dPidPitch + dPidYaw, minSignal, 1700));
    esc2.writeMicroseconds(minMax(throttle - dPidPitch + dPidRoll - dPidYaw, minSignal, 1700));
    esc3.writeMicroseconds(minMax(throttle + dPidPitch + dPidRoll + dPidYaw, minSignal, 1700));
    esc4.writeMicroseconds(minMax(throttle + dPidPitch - dPidYaw, minSignal, 1700));
    esc5.writeMicroseconds(minMax(throttle + dPidPitch - dPidRoll + dPidYaw, minSignal, 1700));
    esc6.writeMicroseconds(minMax(throttle - dPidPitch - dPidRoll - dPidYaw, minSignal, 1700));
  }
  
}

void calculatePID(float actual, float desired, float elapsedTime, float kp, float ki, float kd, float &integral, float &previous, float &finalPID) {
  float error = actual - desired;
  float proporcional = kp * error;
  integral += ki * error;
  float derivate = kd * ((error - previous) / elapsedTime);
  finalPID = proporcional + integral + derivate;
  previous = error;
}

void getBlueData(byte &numReceived, byte resData[], boolean &newData) {
  if (Serial.available() > 0) {
    // Read Bluetooth Data from SofwtwareSerial
    // start message value is '<' and end is '>'
    static boolean recvInProgress = false;
    byte startMarker = '<';
    byte endMarker = '>';
    //byte lineMarker = '\n';
    //byte enterMarker = '\r';
    byte rc;
    static byte ndx = 0; //index of recieve byte

    //Read data from bluetooth
    while (Serial.available() > 0) {
      rc = Serial.read();

      if (recvInProgress == true) {
        if (rc != endMarker) {
          resData[ndx] = rc; //write recieve byte o array
          ndx++;
        }
        else {
          resData[ndx] = '\0'; // terminate the string
          recvInProgress == false;
          newData = true;
          numReceived = ndx;  // save the max index of recieve byte
          ndx = 0;
        }
      }
      else if (rc == startMarker) {
        // when cyckle find marker '<' start write byte to array
        recvInProgress = true;
      }
    }
  }
}

void separateBlueData(byte &numReceived, byte resData[], float &kp, float &ki, float &kd, float &desiredPitch, float &desiredRoll, float &desiredYaw, int &throttle) {
  //Inicialition for NULL value
  desiredPitch = 0;
  desiredRoll = 0;
  desiredYaw = 0;
  throttle = 1000;
  kp = 0;
  ki = 0;
  kd = 0;
  pidIpitch = 0;
  pidIroll = 0;
  pidIYaw = 0;

  //start motor with recieve command
  runDrone = true;

  // seperate byte to command and values drom array full of byte
  for (int i = 0; i < numReceived; i++) {

    if (resData[i] == 'T') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rSpeed[2] = {resData[i + 1], resData[i + 2]};
        int thro = atoi(rSpeed);
        if (thro < 40) {
          throttle = map(atoi(rSpeed), 0, 99, 1000, 1700);
        }
      }
    }

    if (resData[i] == 'C') {
      Serial.println("Drone calibration");
      //delay(10000);
      if (!firstCalib) {
        escCalib(esc1, esc1Pin, esc2, esc2Pin, esc3, esc3Pin, esc4, esc4Pin, esc5, esc5Pin, esc6, esc6Pin, maxSignal, minSignal);
        firstCalib = true;
      }
      Serial.println("The ESCs are calibrated. Lets start :)");
    }

    if (resData[i] == 'X') {
      runDrone = false;
      desiredPitch = 0;
      desiredRoll = 0;
      desiredYaw = 0;
      throttle = 1000;
      kp = 0;
      ki = 0;
      kd = 0;
      esc1.writeMicroseconds(minSignal);
      esc2.writeMicroseconds(minSignal);
      esc3.writeMicroseconds(minSignal);
      esc4.writeMicroseconds(minSignal);
      esc5.writeMicroseconds(minSignal);
      esc6.writeMicroseconds(minSignal);
    }

    if (resData[i] == 'P') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        desiredPitch = map(atof(rInfo), 0, 99, -30, 30);
      }
    }

    else if (resData[i] == 'R') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        desiredRoll = map(atof(rInfo), 0, 99, -30, 30);
      }
    }

    else if (resData[i] == 'Y') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        desiredYaw = map(atof(rInfo), 0, 99, 0, 360);
      }
    }

    else if (resData[i] == 'E') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        kp = atof(rInfo);
      }
    }

    else if (resData[i] == 'I') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        ki = atof(rInfo) / 10 ;
      }
    }

    else if (resData[i] == 'D') {
      if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        kd = atof(rInfo) / 10;
      }
    }
  }
  Serial.print("Pitch ");
  Serial.print(desiredPitch);
  Serial.print(" Roll ");
  Serial.print(desiredRoll);
  Serial.print(" Yaw ");
  Serial.print(desiredYaw);
  Serial.print(" Throttle ");
  Serial.print(throttle);
  Serial.print(" kp ");
  Serial.print(kp);
  Serial.print(" ki ");
  Serial.print(ki);
  Serial.print(" kd ");
  Serial.println(kd);

  newData = false;
  numReceived = 0;
}

static  float minMax(int value, int min_value, int max_value) {
  // fuction which reduce value over interval <min_value; max_value>
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }
  return value;
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
