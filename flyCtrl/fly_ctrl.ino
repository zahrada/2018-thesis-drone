#include <Wire.h>

// declaration of time in millisecond
unsigned long time, time1;
int cal_int;

// set max/min Throttle for ESC and brushless motors calibration
unsigned long timer1, timer2, timer3, timer4, timer5, timer6;
unsigned long timerLoop;

// declarate MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68,
float aX, aY, aZ, gX, gY, gZ, temp;
float accX, accY, gyroX, gyroY, gyroZ;
float pitch, roll, yaw;
float gXOffset, gYOffset, gZOffset;

float error, proporcional, derivate;

//stabilization PID
float kp = 4.5;
float ki = 0;
float kd = 0;
float kpYaw = 5;
float kiYaw = 0;
float kdYaw = 0;
float prevErrorRoll = 0;
float prevErrorPitch = 0;
float prevErrorYaw = 0;
float pidPitch, pidRoll, pidYaw, pidIpitch, pidIroll, pidIyaw;
int pidMax = 300;

//rate PID
float dkp = 1.3;
float dki = 0.00;
float dkd = 6; //6
float dkpYaw = 4.0;
float dkiYaw = 0.02;
float dkdYaw = 0;
float dprevErrorRoll = 0;
float dprevErrorPitch = 0;
float dprevErrorYaw = 0;
float dpidPitch, dpidRoll, dpidYaw,  dpidIpitch, dpidIroll, dpidIyaw;
int dpidMax = 300;

//length pulze for each motor
int esc1, esc2, esc3, esc4, esc5, esc6;

//fuse do calibration before start flying with drone
bool resBool = false;

// UART
byte resData;
byte buffer[2];
int size;
float desiredPitch = 0;
float desiredRoll = 0;
float desiredYaw = 0;
int throttle;

float battery;

void setup() {

  Serial.begin(250000); //start Serial communication
  Serial.println("Start");

  Wire.begin(); //begin the wire comunication
  TWBR = 12;   //Set the I2C clock speed to 400kHz.

  set_gyro_registers();

  readIMU(aX, aY, aZ, gX, gY, gZ, gyroX, gyroY, gyroZ);

  for (cal_int = 0; cal_int < 1000 ; cal_int ++) {                          //Start pwm signal for esc
    readIMU(aX, aY, aZ, gX, gY, gZ, gyroX, gyroY, gyroZ);

    gXOffset += gX;
    gYOffset += gY;
    gZOffset += gZ;
  }

  gXOffset /= 1000;
  gYOffset /= 1000;
  gZOffset /= 1000;

  time = micros(); //Start counting time in milliseconds
  DDRD = DDRD | B11111100;  // this is safer as it sets pins 2 to 7 as outputs

  delayMicroseconds(10000);

  for (cal_int = 0; cal_int < 1250 ; cal_int ++) {                          //Start pwm signal for esc
    PORTD |= B11111100;                                                     //Set digital poort 2- 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00000011;                                                     //Set digital poort 2- 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }
  battery = battery * 0.9 + analogRead(0) * 0.00163;
  Serial.println("Drone is ready");

}

void loop() {

  battery = battery * 0.90 + analogRead(0) * 0.00163; // analogRead(0) * 0.0163 * 0.1 read battery voltage

  // 2000uS
  complementaryFilter(pitch, roll, aX, aY, aZ, gyroX, gyroY, gyroZ);

  readNaviCtrl(throttle, desiredPitch, desiredRoll, desiredYaw, kp, ki, kd);

  // Stabilization PID
  calculatePID(pitch, desiredPitch, kp, ki, kd, pidIpitch,  prevErrorPitch, pidPitch, pidMax);
  calculatePID(roll, desiredRoll, kp, ki, kd, pidIroll,  prevErrorRoll, pidRoll, pidMax);
  calculatePID(yaw, desiredYaw, kpYaw, kiYaw, kdYaw, pidIyaw,  prevErrorYaw, pidYaw, pidMax);

  // Rate PID
  calculatePID(gyroX, -pidPitch, dkp, dki, dkd, dpidIpitch,  dprevErrorPitch, dpidPitch, dpidMax);
  calculatePID(gyroY, -pidRoll, dkp, dki, dkd, dpidIroll,  dprevErrorRoll, dpidRoll, dpidMax);
  calculatePID(gyroZ, gyroZ, dkpYaw, dkiYaw, dkdYaw, dpidIyaw,  dprevErrorYaw, dpidYaw, dpidMax);

  esc1 = minMax((int)(throttle - dpidPitch + dpidYaw), 1000, 1700);
  esc2 = minMax((int)(throttle - dpidPitch + dpidRoll - dpidYaw), 1000, 1700);
  esc3 = minMax((int)(throttle + dpidPitch + dpidRoll + dpidYaw), 1000, 1700);
  esc4 = minMax((int)(throttle + dpidPitch - dpidYaw), 1000, 1700);
  esc5 = minMax((int)(throttle + dpidPitch - dpidRoll + dpidYaw), 1000, 1700);
  esc6 = minMax((int)(throttle - dpidPitch - dpidRoll - dpidYaw), 1000, 1700);

  //create pulze for each motor in frequency 250Hz
  while (micros() - time < 4000);

  time = micros();  // actual time read

  PORTD |= B11111100;                     // Set pin 2-7 HIGH
  if (throttle >= 1050 && battery >= 12.8) {
    if (battery < 16.6) {                                            // compensate battery voltage
      timer1 = (int)minMax((esc1 * 16.8 / battery), 1050, 1700) + time;
      timer2 = (int)minMax((esc2 * 16.8 / battery), 1050, 1700) + time;
      timer3 = (int)minMax((esc3 * 16.8 / battery), 1050, 1700) + time;
      timer4 = (int)minMax((esc4 * 16.8 / battery), 1050, 1700) + time;
      timer5 = (int)minMax((esc5 * 16.8 / battery), 1050, 1700) + time;
      timer6 = (int)minMax((esc6 * 16.8 / battery), 1050, 1700) + time;
    }
    else {
      timer1 = esc1 + time;
      timer2 = esc2 + time;
      timer3 = esc3 + time;
      timer4 = esc4 + time;
      timer5 = esc5 + time;
      timer6 = esc6 + time;
    }
  }
  else {
    timer1 = 980 + time;
    timer2 = 980 + time;
    timer3 = 980 + time;
    timer4 = 980 + time;
    timer5 = 980 + time;
    timer6 = 980 + time;
  }
  // 1000 uS -2000uS
  //Serial.println("readImu");
  //Serial.println(micros());
  readIMU(aX, aY, aZ, gX, gY, gZ, gyroX, gyroY, gyroZ);

  while (PORTD >= B00000011) {            // stay in the loop, while pin 2-7 HIGH
    timerLoop = micros();
    if (timer1 <= timerLoop) {
      PORTD &= B01111111;                 // Set pin 7 LOW
    }
    if (timer2 <= timerLoop) {
      PORTD &= B10111111;                 // Set pin 6 LOW
    }
    if (timer3 <= timerLoop) {
      PORTD &= B11011111;                 // Set pin 5 LOW
    }
    if (timer4 <= timerLoop) {
      PORTD &= B11101111;                 // Set pin 4 LOW
    }
    if (timer5 <= timerLoop) {
      PORTD &= B11110111;                 // Set pin 3 LOW
    }
    if (timer6 <= timerLoop) {
      PORTD &= B11111011;                 // Set pin 2 LOW
    }
  }
}

void readIMU(float &aX, float &aY, float &aZ, float &gX, float &gY, float &gZ, float &gyroX, float &gyroY, float &gyroZ) {
  // read raw data from IMU
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  aX = Wire.read() << 8 | Wire.read();
  aY = Wire.read() << 8 | Wire.read();
  aZ = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gX = Wire.read() << 8 | Wire.read();
  gY = Wire.read() << 8 | Wire.read();
  gZ = Wire.read() << 8 | Wire.read();

  aX /= 4096.0;
  aY /= 4096.0;
  aZ /= 4096.0;

  gX /= 65.5;
  gY /= 65.5;
  gZ /= 65.5;

  gyroX = gyroX * 0.7 + gX * 0.3;
  gyroY = gyroY * 0.7 + gY * 0.3;
  gyroZ = gyroZ * 0.7 + gZ * 0.3;
}

void complementaryFilter(float &pitch, float &roll, float aX, float aY, float aZ, float gyroX, float gyroY, float gyroZ) {
  gyroX -= gXOffset;
  gyroY -= gYOffset;
  gyroZ -= gZOffset;

  //Complementary filter
  accX = atan2((aY) , sqrt(pow(aX, 2) + pow(aZ, 2))) * (180 / M_PI);
  accY = atan2(-1 * (aX) , sqrt(pow(aY, 2) + pow(aZ, 2)))  * (180 / M_PI);

  pitch = 0.996 * (pitch + gyroX  * 0.004) + 0.004 * accX;
  roll = 0.996 * (roll + gyroY * 0.004) + 0.004 * accY;

  pitch -= roll * sin(gyroZ * 0.0000698);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  roll += pitch * sin(gyroZ * 0.0000698);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
}

static  int minMax(int value, int min_value, int max_value) {
  // fuction which reduce value over interval <min_value; max_value>
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }
  return value;
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

void calculatePID(float actual, float desired, float kp, float ki, float kd, float &integral, float &previous, float &finalPID, float pidMax) {
  error = actual - desired;
  integral += ki * error;
  finalPID = kp * error + integral + kd * (error - previous);
  previous = error;
  finalPID = minMax(finalPID, -pidMax, pidMax);
}

void readNaviCtrl(int &throttle, float &desiredPitch, float &desiredRoll, float &desiredYaw, float &kp, float &ki, float &kd) {
  while (Serial.available() > 0) {                  // if data is incoming
    resData = Serial.read();                        // read first byte
    if (resData == 'X') {
      desiredPitch = 0;
      desiredRoll = 0;
      desiredYaw = 0;
      throttle = 0;
      pidIpitch = 0;
      pidIroll = 0;
      pidIyaw = 0;
      dpidIpitch = 0;
      dpidIroll = 0;
      dpidIyaw = 0;
    }
    if (resData == 'T') {
      Serial.readBytes(buffer, 2);                  // read byte and store in array buffer
      throttle = map(atoi(buffer), 0, 99, 1000, 1700);                      // convert byte array to integer
    }
    if (resData == 'P') {
      Serial.readBytes(buffer, 2);
      desiredPitch = map(atof(buffer), 0, 99, -25, 25);                  // convert byte array to float
    }
    if (resData == 'R') {
      Serial.readBytes(buffer, 2);
      desiredRoll = map(atof(buffer), 0, 99, -25, 25);
    }
    if (resData == 'Y') {
      Serial.readBytes(buffer, 2);
      desiredYaw = map(atof(buffer), 0, 99, 0, 360);
    }
  }
}

void set_gyro_registers() {
  //Setup the MPU-6050
  writeRegister(0x68, 0x6B, 0x00); //Set the register bits as 00000000 to activate the gyro

  writeRegister(0x68, 0x1B, 0x08); //Set the register bits as 00001000 (500dps full scale)

  writeRegister(0x68, 0x1C, 0x10); //Set the register bits as 00010000 (+/- 8g full scale range)

  writeRegister(0x68, 0x1A, 0x03); //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
}

void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}
