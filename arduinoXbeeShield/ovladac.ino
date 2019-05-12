#include <SoftwareSerial.h> // include the SoftwareSerial library so you can use its functions

// set pins for hc-06 bluetooth module
#define rxPin 11
#define txPin 10

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0)
  {
    byte b = Serial.read();
    mySerial.print(b);
  }


}
