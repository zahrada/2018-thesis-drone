#include <Wire.h>

// declaration of time in millisecond
unsigned long time;
float elapsedTime, timePrev;

// set max/min Throttle for ESC and brushless motors calibration
const int maxSignal = 2000;
const int minSignal = 1000;

// declarate MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68,
float aX, aY, aZ, gX, gY, gZ, temp;
float accX, accY, gyroX, gyroY;
float pitch, roll, yaw;

void setup() {

  Serial.begin(250000); //start Serial communication
  Serial.println("Start");

  DDRD |= B11111100; // Set pin 2-7 as a OUTPUT

  Wire.begin(); //begin the wire comunication
  TWBR = 12;   //Set the I2C clock speed to 400kHz.
  Wire.beginTransmission(0x68); //begin transmissin with IMU on address 0x68 (AD0 is LOW)
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  readIMU(pitch, roll);

  time = micros(); //Start counting time in milliseconds
  Serial.println("Drone is ready");
}

void loop() {

  timePrev = time;  // the previous time is stored before the actual time read
  //readIMU(pitch, roll);
  // read raw data from IMU
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  while (Wire.available() < 14) {
    aX = Wire.read() << 8 | Wire.read();
    aY = Wire.read() << 8 | Wire.read();
    aZ = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gX = Wire.read() << 8 | Wire.read();
    gY = Wire.read() << 8 | Wire.read();
    gZ = Wire.read() << 8 | Wire.read();
  }

  aX /= 16384.0;
  aY /= 16384.0;
  aZ /= 16384.0;
  gX /= 131.0;
  gY /= 131.0;
  gZ /= 131.0;

  //Complementary filter
  accX = atan2((aY) , sqrt(pow(aX, 2) + pow(aZ, 2))) * (180 / M_PI);
  accY = atan2(-1 * (aX) , sqrt(pow(aY, 2) + pow(aZ, 2)))  * (180 / M_PI);

  gyroX = pitch + gX  * elapsedTime / 10e6;
  gyroY = roll + gY * elapsedTime / 10e6;

  pitch = 0.99 * gyroX + 0.01 * accX;
  roll = 0.99 * gyroY + 0.01 * accY;

  time = micros();  // actual time read
  elapsedTime = (time - timePrev);

  Serial.println(elapsedTime);

}

void readIMU(float &pitch, float &roll) {
  float aX, aY, aZ, gX, gY, gZ, temp;
  // read raw data from IMU
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  while (Wire.available() < 14) {
    aX = Wire.read() << 8 | Wire.read();
    aY = Wire.read() << 8 | Wire.read();
    aZ = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gX = Wire.read() << 8 | Wire.read();
    gY = Wire.read() << 8 | Wire.read();
    gZ = Wire.read() << 8 | Wire.read();
  }

  aX /= 16384.0;
  aY /= 16384.0;
  aZ /= 16384.0;
  gX /= 131.0;
  gY /= 131.0;
  gZ /= 131.0;

  //Complementary filter
  float accX = atan2((aY) , sqrt(pow(aX, 2) + pow(aZ, 2))) * (180 / M_PI);
  float accY = atan2(-1 * (aX) , sqrt(pow(aY, 2) + pow(aZ, 2)))  * (180 / M_PI);

  float gyroX = pitch + gX  * elapsedTime / 10e6;
  float gyroY = roll + gY * elapsedTime / 10e6;

  pitch = 0.99 * gyroX + 0.01 * accX;
  roll = 0.99 * gyroY + 0.01 * accY;

}
/*
  void escCalib(int maxSignal, int minSignal)
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
*/
