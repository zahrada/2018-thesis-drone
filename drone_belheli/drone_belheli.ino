/*ESC calibration sketch; author: ELECTRONOOBS */
#include <Servo.h>
#include <Wire.h>
#include "MPU9250.h" //include library with gyro/akce/mag
#include "MahonyAHRS.h" // include library to filter data of gyro/akce/mag to roll, pich, yaw

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

#define ESC1_PIN 7
#define ESC2_PIN 6
#define ESC3_PIN 5
#define ESC4_PIN 4
#define ESC5_PIN 3
#define ESC6_PIN 2

int DELAY = 1000;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
Servo esc5;
Servo esc6;

// class for filter data from gyro/akce/mag
Mahony filter;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
float ax, ay, az, gx, gy, gz, mx, my, mz, deltat, roll, pitch, heading;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  while (!Serial) {}
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.print("IMU initialization unsuccessful");
    Serial.print("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.print(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
  filter.begin(50);

  Serial.println("Don't forget to subscribe!");
  Serial.println("ELECTRONOOBS ESC calibration...");
  Serial.println(" ");
  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");
  esc1.attach(ESC1_PIN);
  delay(10);
  esc2.attach(ESC2_PIN);
  delay(10);
  esc3.attach(ESC3_PIN);
  delay(10);
  esc4.attach(ESC4_PIN);
  delay(10);
  esc5.attach(ESC5_PIN);
  delay(10);
  esc6.attach(ESC6_PIN);
  delay(10);

  Serial.print("ESCs calibration");

  delay(2000);
  esc1.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  esc1.writeMicroseconds(MIN_SIGNAL);
  delay(2000);

  esc2.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  esc2.writeMicroseconds(MIN_SIGNAL);
  delay(2000);

  esc3.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  esc3.writeMicroseconds(MIN_SIGNAL);
  delay(2000);

  esc4.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  esc4.writeMicroseconds(MIN_SIGNAL);
  delay(2000);

  esc5.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  esc5.writeMicroseconds(MIN_SIGNAL);
  delay(2000);

  esc6.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  esc6.writeMicroseconds(MIN_SIGNAL);
  delay(2000);

  delay(5000);
  Serial.println("The ESCs are calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
  delay(5000);
}

void loop() {

  // read the sensor
  IMU.readSensor();
  //***************************************************************************************************************
  // Mahonny Filtracion********************************************************************************************
  // Similar to Madgwick scheme but uses proportional and integral filtering on
  // the error between estimated reference vectors and measured ones.
  // short name local variable for readability
  filter.update(IMU.getGyroX_rads() * (180 / M_PI), IMU.getGyroY_rads() * (180 / M_PI), IMU.getGyroZ_rads() * (180 / M_PI), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());

  //***************************************************************************************************************************
  // Compute direction to North from magnetometr not used *********************************************************************
  float angl_north = angle_north( IMU.getMagX_uT(), IMU.getMagY_uT(), 0.07);
  //***************************************************************************************************************************
  // Compute roll, pitch, yaw
  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  /*
    if (Serial.available() > 0)
    {
      int speed = Serial.parseInt();
      if (speed > 999)
      {
  */
  int speed = 100;
  balanceDrone(speed, roll, pitch);
  /*
      }

    }
  */
}
static float angle_north( float mx, float my, float magDeclinRad)
{
  //Calculate angle to North from magnetometr mx, my in degrees
  //Fix by magnetic Declination of lacation
  //
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
  // int speed_upgrade = speed_change * angle / 60;
  speed_second = speed_change * (60 - angle) / 60;
  speed_first = speed_change * angle / 60;
}

static void setSpeedDrone(int &speed)
{
  //Set speed to all 6 motors
  esc1.writeMicroseconds(speed);
  esc2.writeMicroseconds(speed);
  esc3.writeMicroseconds(speed);
  esc4.writeMicroseconds(speed);
  esc5.writeMicroseconds(speed);
  esc6.writeMicroseconds(speed);
}

void flyUp(int &speed, float &high)
{


}

static void balanceDrone(int &speed, float &roll, float &pitch)
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

  // <-180;180>
  float balanc_angle = atan2(pitch, roll) * 180 / PI + 180;

  if (balanc_angle > 0 && balanc_angle < 30 || balanc_angle > 330 && balanc_angle < 360)
  {
    Serial.print("Motor 2-3");
    Serial.print("\t");
    Serial.print(balanc_angle);
    int speed_first;
    int speed_second;
    if (balanc_angle > 0 && balanc_angle < 30)
    {
      balanc_angle += 30;
    }
    else
    {
      balanc_angle -= 330;
    }

    controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speed);
    
      esc1.writeMicroseconds(MIN_SIGNAL);
      esc2.writeMicroseconds(MIN_SIGNAL + speed_first);
      esc3.writeMicroseconds(MIN_SIGNAL + speed_second);
      esc4.writeMicroseconds(MIN_SIGNAL);
      esc5.writeMicroseconds(MIN_SIGNAL);
      esc6.writeMicroseconds(MIN_SIGNAL);
      Serial.print("\n");

    }
    else if (balanc_angle > 30 && balanc_angle < 90)
    {
      Serial.print("Motor 1-2");
      Serial.print("\t");
      Serial.print(balanc_angle);

      int speed_first;
      int speed_second;
      balanc_angle -= 30;

      controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speed);

      esc1.writeMicroseconds(MIN_SIGNAL + speed_first);
      esc2.writeMicroseconds(MIN_SIGNAL + speed_second);
      esc3.writeMicroseconds(MIN_SIGNAL);
      esc4.writeMicroseconds(MIN_SIGNAL);
      esc5.writeMicroseconds(MIN_SIGNAL);
      esc6.writeMicroseconds(MIN_SIGNAL);

      Serial.print("\n");
    }
    else if (balanc_angle > 90 && balanc_angle < 150)
    {
      Serial.print("Motor 1-6");
      Serial.print("\t");
      Serial.print(balanc_angle);


      int speed_first;
      int speed_second;
      balanc_angle -= 90;

      controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speed);

      esc1.writeMicroseconds(MIN_SIGNAL + speed_second);
      esc2.writeMicroseconds(MIN_SIGNAL);
      esc3.writeMicroseconds(MIN_SIGNAL);
      esc4.writeMicroseconds(MIN_SIGNAL);
      esc5.writeMicroseconds(MIN_SIGNAL);
      esc6.writeMicroseconds(MIN_SIGNAL + speed_first);

      Serial.print("\n");
    }
    else if (balanc_angle > 150 && balanc_angle < 210)
    {
      Serial.print("Motor 5-6");
      Serial.print("\t");
      Serial.print(balanc_angle);

      int speed_first;
      int speed_second;
      balanc_angle -= 150;

      controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speed);

      esc1.writeMicroseconds(MIN_SIGNAL);
      esc2.writeMicroseconds(MIN_SIGNAL);
      esc3.writeMicroseconds(MIN_SIGNAL);
      esc4.writeMicroseconds(MIN_SIGNAL);
      esc5.writeMicroseconds(MIN_SIGNAL + speed_first);
      esc6.writeMicroseconds(MIN_SIGNAL + speed_second);

      Serial.print("\n");
    }
    else if (balanc_angle > 210 && balanc_angle < 270)
    {
      Serial.print("Motor 5-4");
      Serial.print("\t");
      Serial.print(balanc_angle);

      int speed_first;
      int speed_second;
      balanc_angle -= 210;

      controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speed);

      esc1.writeMicroseconds(MIN_SIGNAL);
      esc2.writeMicroseconds(MIN_SIGNAL);
      esc3.writeMicroseconds(MIN_SIGNAL);
      esc4.writeMicroseconds(MIN_SIGNAL + speed_first);
      esc5.writeMicroseconds(MIN_SIGNAL + speed_second);
      esc6.writeMicroseconds(MIN_SIGNAL);

      Serial.print("\n");
    }
    else if (balanc_angle > 270 && balanc_angle < 330)
    {
      Serial.print("Motor 4-3");
      Serial.print("\t");
      Serial.print(balanc_angle);

      int speed_first;
      int speed_second;
      balanc_angle -= 270;

      controlSpeedBetween2Motors(speed_first, speed_second,  balanc_angle, speed);

      esc1.writeMicroseconds(MIN_SIGNAL);
      esc2.writeMicroseconds(MIN_SIGNAL);
      esc3.writeMicroseconds(MIN_SIGNAL + speed_first);
      esc4.writeMicroseconds(MIN_SIGNAL + speed_second);
      esc5.writeMicroseconds(MIN_SIGNAL);
      esc6.writeMicroseconds(MIN_SIGNAL);

      Serial.print("\n");
    }
  }

  static void rotateDrone(float & heading, int &speed)
  {
    // control ESC by IMU, rotate to north by heading to navigate direction between motor1 and motor2
    // heading data is <0,360>
    // if heading is less than 10 and more than 350, nothnig happend, drone is stil ballancing
    // if heading is more than 10 and less than 180, drone turn by CCW
    // if heading is more than 180 and less than 350, drone turn by CW
    // global value esc1 -> esc6 by Servo.h
    // global value MIN_SIGNAL = 1000

    if (heading < 180 && heading > 10)
    {
      while (heading < 180 && heading > 10)
      {
        esc1.writeMicroseconds(MIN_SIGNAL + speed);
        esc2.writeMicroseconds(MIN_SIGNAL);
        esc3.writeMicroseconds(MIN_SIGNAL + speed);
        esc4.writeMicroseconds(MIN_SIGNAL);
        esc5.writeMicroseconds(MIN_SIGNAL + speed);
        esc6.writeMicroseconds(MIN_SIGNAL);
      }
    }
    else if (heading > 180 && heading < 350)
    {
      while (heading > 180 && heading < 350)
      {
        esc1.writeMicroseconds(MIN_SIGNAL);
        esc2.writeMicroseconds(MIN_SIGNAL + speed);
        esc3.writeMicroseconds(MIN_SIGNAL);
        esc4.writeMicroseconds(MIN_SIGNAL + speed);
        esc5.writeMicroseconds(MIN_SIGNAL);
        esc6.writeMicroseconds(MIN_SIGNAL + speed);
      }
    }
    else
    {
      
    }
  }
