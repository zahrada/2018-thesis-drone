// include the SoftwareSerial library so you can use its functions:
#include <SoftwareSerial.h>

#define rxPin 11
#define txPin 10

// set up a new serial port
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

void setup()  {
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Arduino is ON");
}

void loop() {
  boolean  newData = false;
  const byte numResByte = 32;
  byte resData[numResByte];
  byte numReceived = 0;
  char resCommand;
  int resSpeed, resInfo;
  
  getBlueData(mySerial, numReceived, resData, newData);
  // Process data from bluetooth to command drone
  if (newData == true) {
    separateBlueData(numReceived, resData, resCommand, resSpeed, resInfo);
    doCommandDrone(resCommand, resSpeed, resInfo);
  }
  delay(100);
}

void getBlueData(SoftwareSerial &mySerial, byte &numReceived, byte resData[], boolean &newData) {
  // Read Bluetooth Data from HC-05
  // start message value is 's' and end is 'e'
  static boolean recvInProgress = false;
  byte startMarker = '<';
  byte endMarker = '>';
  byte lineMarker = '\n';
  byte enterMarker = '\r';
  byte rc;
  static byte ndx = 0;

  //Read data from bluetooth
  while (mySerial.available() > 0) {
    rc = mySerial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        resData[ndx] = rc;
        ndx++;

        //mySerial.print(char(rc));
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
    if (resData[i] == 'B' || resData[i] == 'F' || resData[i] == 'R')
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
        mySerial.println("Try again");
      }
    }
    // Read Info of Command 01-99
    if (resData[i] == 'A') {

        if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
          byte rInfo[2] = {resData[i + 1], resData[i + 2]};
          resInfo = atoi(rInfo);
        }
        else{
          mySerial.println("Try again");
        }
    }
  }

  //Finnal conditions
  if(resInfo == 0){
    resSpeed = 0;
    mySerial.println("Try again");
  }
    if(resSpeed == 0){
    resInfo = 0;
    mySerial.println("Try again");
  }
}

void doCommandDrone(char &resCommand, int &resSpeed, int &resInfo) {
  //Command drone
  switch (resCommand) {

    case 'B': // 66 = b Ascii table
      mySerial.print("balance ");
      mySerial.print(resSpeed);
      mySerial.print(" info ");
      mySerial.println(resInfo);

      break;

    case 'F':// 70 = f Ascii table
      mySerial.print("flying ");
      mySerial.print(resSpeed);
      mySerial.print(" info ");
      mySerial.println(resInfo);
      break;

    case 'R': // 82 = r Ascii table
      mySerial.print("rotate ");
      mySerial.print(resSpeed);
      mySerial.print(" info ");
      mySerial.println(resInfo);
      break;

    default:
      // v případě přijetí ostatních znaků
      // vytiskneme informaci o neznámé zprávě
      mySerial.println("Try again");
  }
}
/*
   void loop() {
  newData = false;
  recvInProgress = false;

  //Read data from bluetooth
  while (mySerial.available() > 0) {
    rc = mySerial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        resData[ndx] = rc;
        ndx++;

        //mySerial.print(char(rc));
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

  // Process data from bluetooth to command drone
  if (newData == true) {
    for (int i = 0; i < numReceived; i++) {
      //Type of command
      if (resData[i] == 'B' || resData[i] == 'F' || resData[i] == 'R')
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
          mySerial.println("Try again");
        }
      }
      // Read Info of Command 01-99
      if (resData[i] == 'A') {
        if (isDigit(char(resData[i + 1])) && isDigit(char(resData[i + 2]))) {
        byte rInfo[2] = {resData[i + 1], resData[i + 2]};
        resInfo = atoi(rInfo);
        }
        else {
          resInfo = 0;
          mySerial.println("Try again");
        }
      }
    }

    //Command drone
    switch (resCommand) {

      case 'B': // 66 = b Ascii table
        mySerial.print("balance ");
        mySerial.print(resSpeed);
        mySerial.print(" info ");
        mySerial.println(resInfo);

        break;

      case 'F':// 70 = f Ascii table
        mySerial.print("flying ");
        mySerial.print(resSpeed);
        mySerial.print(" info ");
        mySerial.println(resInfo);
        break;

      case 'R': // 82 = r Ascii table
        mySerial.print("rotate ");
        mySerial.print(resSpeed);
        mySerial.print(" info ");
        mySerial.println(resInfo);
        break;

      default:
        // v případě přijetí ostatních znaků
        // vytiskneme informaci o neznámé zprávě
        mySerial.println("Neznamy prikaz.");
    }
  }
  delay(100);
  }*/
