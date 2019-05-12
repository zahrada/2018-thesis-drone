#include <Wire.h>
#include <VL53L0X.h>
/*
             /\     /\
             ||     ||
           dist1  dist2
              1    2

  < - dist6 6  (dist7)  3  dist3 ->

              5    4
           dist5  dist4
             ||     ||
             \/     \/
*/
//define distance if obstacle
#define maxDistObstacle 1300 // 1 meter

//define shut down pin on laser
#define XSHUT_pin7 12
#define XSHUT_pin6 11
#define XSHUT_pin5 10
#define XSHUT_pin4 9
#define XSHUT_pin3 8
#define XSHUT_pin2 7
#define XSHUT_pin1 6

//ADDRESS_DEFAULT 0b0101001 or 41
//#define Sensor1_newAddress 41 not required address change
#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
#define Sensor5_newAddress 45
#define Sensor6_newAddress 46
#define Sensor7_newAddress 47


VL53L0X Sensor1, Sensor2,  Sensor3, Sensor4, Sensor5, Sensor6, Sensor7;

int dist1, dist2, dist3, dist4, dist5, dist6, dist7;

void setup() { /*WARNING*/
  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
  pinMode(XSHUT_pin5, OUTPUT);
  pinMode(XSHUT_pin6, OUTPUT);
  pinMode(XSHUT_pin7, OUTPUT);

  Serial.begin(9600);

  Wire.begin();
  //Change address of sensor and power up next one

  pinMode(XSHUT_pin7, INPUT);
  delay(10);
  Sensor7.setAddress(Sensor7_newAddress);
  pinMode(XSHUT_pin6, INPUT);
  delay(10);
  Sensor6.setAddress(Sensor6_newAddress);
  pinMode(XSHUT_pin5, INPUT);
  delay(10);
  Sensor5.setAddress(Sensor5_newAddress);
  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);
  pinMode(XSHUT_pin3, INPUT);
  delay(10);
  Sensor3.setAddress(Sensor3_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10);

  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  Sensor4.init();
  Sensor5.init();
  Sensor6.init();
  Sensor7.init();

  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor3.setTimeout(500);
  Sensor4.setTimeout(500);
  Sensor5.setTimeout(500);
  Sensor6.setTimeout(500);
  Sensor7.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Sensor1.startContinuous(500);
  Sensor2.startContinuous(500);
  Sensor3.startContinuous(500);
  Sensor4.startContinuous(500);
  Sensor5.startContinuous(500);
  Sensor6.startContinuous(500);
  Sensor7.startContinuous(500);

  delay(1000);
}

void loop()
{
  /*
    dist1 = Sensor1.readRangeContinuousMillimeters();
    dist2 = Sensor2.readRangeContinuousMillimeters();
    dist3 = Sensor3.readRangeContinuousMillimeters();
    dist4 = Sensor4.readRangeContinuousMillimeters();
    dist5 = Sensor5.readRangeContinuousMillimeters();
    dist6 = Sensor6.readRangeContinuousMillimeters();
    dist7 = Sensor7.readRangeContinuousMillimeters();

    Serial.print(dist1);
    Serial.print(" ");
    Serial.print(dist2);
    Serial.print(" ");
    Serial.print(dist3);
    Serial.print(" ");
    Serial.print(dist4);
    Serial.print(" ");
    Serial.print(dist5);
    Serial.print(" ");
    Serial.print(dist6);
    Serial.print(" ");
    Serial.println(dist7);
  */

  dist1 = 1;
  dist2 = 2000;
  dist3 = 3000;
  dist4 = 2000;
  dist5 = 5000;
  dist6 = 2000;
  dist7 = 7000;

  if (dist1 < maxDistObstacle) {
    Serial.print("1");
  }
  if (dist2 < maxDistObstacle) {
    Serial.print("2");
  }
  if (dist3 < maxDistObstacle) {
    Serial.print("2");
  }
  if (dist4 < maxDistObstacle) {
    Serial.print("3");
  }
  if (dist5 < maxDistObstacle) {
    Serial.print("4");
  }
  if (dist6 < maxDistObstacle) {
    Serial.print("4");
  }
  if (dist7 < maxDistObstacle) {
    Serial.print("0");
  }

}
