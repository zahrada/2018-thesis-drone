#include <Wire.h>
#include <VL53L0X.h>

//define distance if obstacle
#define maxDistObstacle 1000 // 1 meter

//#define XSHUT_pin6 not required for address change
#define XSHUT_pin7 8
#define XSHUT_pin6 7
#define XSHUT_pin5 6
#define XSHUT_pin4 5
#define XSHUT_pin3 4
#define XSHUT_pin2 3
#define XSHUT_pin1 2

//ADDRESS_DEFAULT 0b0101001 or 41
//#define Sensor1_newAddress 41 not required address change
#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
#define Sensor5_newAddress 45
#define Sensor6_newAddress 46
#define Sensor7_newAddress 47

VL53L0X Sensor1, Sensor2,  Sensor3, Sensor4, Sensor5, Sensor6, Sensor7; // Sensor7 is Altitude

int dist1, dist2, dist3, dist4, dist5, dist6, dist7;

void setup()
{ /*WARNING*/
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

}

void loop()
{
  dist1 = Sensor1.readRangeContinuousMillimeters();
  dist2 = Sensor2.readRangeContinuousMillimeters();
  dist3 = Sensor3.readRangeContinuousMillimeters();
  dist4 = Sensor4.readRangeContinuousMillimeters();
  dist5 = 1000;
  dist6 = 1000;
  dist7 = 1000;


  if ( dist1 < maxDistObstacle || dist2 < maxDistObstacle || dist3 < maxDistObstacle || dist4 < maxDistObstacle) {
    Serial.println("1");
  }
  else {
    Serial.println("0");
  }

}

static void obstacleDrone(int dist1, int dist2, int dist3, int dist4, int dist5, int dist6) {
  if (dist1 < maxDistObstacle) {

  }
  else if (dist2 < maxDistObstacle) {

  }
  else if (dist3 < maxDistObstacle) {

  }
  else if (dist4 < maxDistObstacle) {

  }
  else if (dist5 < maxDistObstacle) {

  }
  else if (dist6 < maxDistObstacle) {

  }
}
