/*     Open Lidar Project Code
 *     Author: Adam Sampson
 *     
 *     License: The software in this project is licensed under the GPLv3 lisence. The 
 *     hardware is licensed under the TARP Open Hardware License v1.0. 
 *     
 *     Acknowledgements - This project uses commonly available tutorials and libraries. 
 *     The code has been modified to meet the needs of the project. This project has been 
 *     made possible by the generous work of everyone in the open source community. As such
 *     it is only fair to make this project open source.
 *     
 *     Disclaimer - This code comes without any warranty or guarantee. The software and 
 *     hardware schematics are provided AS-IS and are for reference use only. Any user is 
 *     responsible for verifying the capabilities within their needs. The author is not 
 *     liable for any damages incurred through the use of this information.
 *     
 *     SD card attached to the SPI BUss
 *     ** NOTE: This REQUIRES >100 mA of current. This cannot run off the trinket pro 5V pin.
 *       MOSI - Pin 11
 *       MISO - Pin 12
 *       CLK - Pin 13
 *       CS - Pin 10
 *     
 *     Bluetooth Connected to the TX/RX pins
 *       TX - Pin 1 on Arduino to RX on Bluetooth Module
 *       RX - Pin 0 on Artuino to TX on Bluetooth Module
 *     
 *     Hall Effect digital out connected to the A0 pin
 *       D0 - Pin A2
 *     
 *     Motor Common Pin
 *      SLP/RST - Pin 8 //High to enable A4988 board, Low to sleep
 *      MS1 - Pin 9
 *      MS2 - Pin A3
 *          **************************
 *          *              MS1   MS2 * 
 *          *Full Step     Low   Low *
 *          *Half Step     High  Low *
 *          *Quarter Step  Low   High*
 *          *Eighth Step   High  High*
 *          **************************
 *     
 *     Motor 1 connected to digital output pins
 *       Step - Pin 4
 *       Direction - Pin 3
 *     
 *     Motor 2 connected to digital output pins
 *       Step - Pin 6
 *       Direction - Pin 5
 *     
 *     SF30-B connected to digital input/output pins through SoftwareSerial
 *       ? - Pin A0 as ??
 *       ? - Pin A1 as ??
 */

#include <SPI.h>
#include <SD.h>

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINTLN(x)  Serial.println (x)
 #define DEBUG_PRINT(x) Serial.print (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

#define BAUD (115200)
// define an array of characters to read serial into
String serialData;

// define pins numbers
const int slpPin = 8;
const int MS1Pin = 9;
const int MS2Pin = A3;

const int stepPin1 = 4; // motor 1 step pin
const int dirPin1 = 3;  // motor 1 direction pin
const int stepPin2 = 6; // motor 2 step pin 
const int dirPin2 = 5;  // motor 2 direction pin
const int digHall = A2; // digital hall effect pin

// define motor profile
const int riseDelay1 = 125;
const int fallDelay1 = 125;
const int riseDelay2 = 1200; 
const int fallDelay2 = 1200;

int myYawSteps = 400*8;  // This is a 400 step motor with Eighth Steps
int myPitchSteps = 400*8;  // This is a 400 step motor with Eight Steps

// define SD card variables
File myFile;
boolean mySdStatus = 0;
const int slaveSelect = 10; // Pin 10 is the chip select pin

void setup() {
  // put your setup code here, to run once:
  // Set serial for bluetooth (or FTDI)
  Serial.begin(BAUD);

  // Set motors to sleep mode to conserve power
  pinMode(slpPin,OUTPUT);
  digitalWrite(slpPin,LOW);

  // Set common motor pins to default Eighth Step Mode 
  pinMode(MS1Pin,OUTPUT);
  pinMode(MS2Pin,OUTPUT);
  digitalWrite(MS1Pin,HIGH);
  digitalWrite(MS2Pin,HIGH);
  
  // Sets the motor driver pins as Outputs
  pinMode(dirPin1,OUTPUT);
  pinMode(stepPin1,OUTPUT); 
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin2,OUTPUT); 

  // Sets the Hall Effect pin at Input
  pinMode(digHall,INPUT);

  // Set up the SD card
  pinMode(slaveSelect,OUTPUT); //attempting to prevent hardware from setting Arduino as Slave
  DEBUG_PRINTLN("Initilazing SD card...");
  if (!SD.begin(4)) {
    DEBUG_PRINTLN("Initialization Failed.");
    mySdStatus = 0;
  }
  else {
    DEBUG_PRINTLN("Initialization Complete.");
    mySdStatus = 1; 
  }
  if (mySdStatus == 1) {
    if (SD.exists("DEBUG.txt")) {
      DEBUG_PRINTLN("DEBUG.txt exists, deleting file");
      SD.remove("DEBUG.txt");
      if (SD.exists("DEBUG.txt")) {
        DEBUG_PRINTLN("Delete failed.");
      }
      else {
        DEBUG_PRINTLN("Delete succeeded.");
      }
    }

    DEBUG_PRINTLN("Creating fresh DEBUG.txt");
    myFile = SD.open("DEBUG.txt", FILE_WRITE);
    myFile.close();
  
    if (SD.exists("DEBUG.txt")) {
      DEBUG_PRINTLN("DEBUG.txt created");
    }
  }
  
}

void loop() {
  // Set variables
  unsigned long microcounter;
  microcounter = micros();

  //delay(10000); // 10 second delay to get hands out of the way

  //Listen for bluetooth commands
  while(Serial.available()) {
    serialData = Serial.readString(); //read incoming data as a string
    DEBUG_PRINT("OL recieved: ");
    DEBUG_PRINTLN(serialData);
  }

  //Decide what to do with the command recieved
  if(serialData == "TMTM") {
    for (int j = 0; j < 1; j++) {  //test the motor j times.
    // Make sure they aren't sleeping
    digitalWrite(slpPin,HIGH);
    delay(500);
    // Test motor 1 for 20 steps at default quarter step of 0.9deg
    TestMotor(1,myPitchSteps);
    delay(500);
    //Test motor 2 for 20 steps at default quarter step of 0.9deg
    TestMotor(2,myYawSteps);
    delay(500);
    }

    // Put the motor to sleep to save power
    digitalWrite(slpPin,LOW);

    //Unset the command string so we don't repeat next loop
    serialData = "";
  }
  else if (serialData == "SRSR") {
    scanRoom();  
  }
}

boolean readHall() {
  boolean digHallState = 0;
  return digitalRead(digHall);
  DEBUG_PRINT("Digital Hall State: ");
  DEBUG_PRINTLN(digHallState);
}

void StepMotorForward(int Motor){
  //temp variables
  int tempStepPin;
  int tempDirPin;

  switch(Motor) {
    case 1:
      tempStepPin = stepPin1;
      tempDirPin = dirPin1;
      digitalWrite(tempDirPin,HIGH);
      digitalWrite(tempStepPin,HIGH);
      delayMicroseconds(riseDelay1);
      digitalWrite(tempStepPin,LOW);
      delayMicroseconds(fallDelay1);
      break;
    case 2:
      tempStepPin = stepPin2;
      tempDirPin = dirPin2;
      digitalWrite(tempDirPin,HIGH);
      digitalWrite(tempStepPin,HIGH);
      delayMicroseconds(riseDelay2);
      digitalWrite(tempStepPin,LOW);
      delayMicroseconds(fallDelay2);
      break;
    default: //we only have two motors...cancel if something else selected
      return;
      break;
  }
  
  

}

void TestMotor(int Motor, int Steps) {
  //Make sure the motors are not sleeping
  digitalWrite(slpPin,HIGH);

  //
  for (int i=0; i <= Steps; i++) {
    StepMotorForward(Motor);
  }
}



void TestWriteToSD() {
  DEBUG_PRINT("SD Status: ");
  DEBUG_PRINT(mySdStatus);
  DEBUG_PRINT("  ");
  
  // re-open the file for writing
  myFile = SD.open("DEBUG.txt", FILE_WRITE);

  // if the file opened okay, write to it
  if (myFile) {
    DEBUG_PRINTLN("Printing to file");
    myFile.print("SD Status: ");
    myFile.println(mySdStatus);
    myFile.close(); // Make sure to close the file to save changes.
  }
  else {
    DEBUG_PRINTLN("Error opening file to write.");
  }
}

void scanRoom() {
   //Make sure the motors are not sleeping
  digitalWrite(slpPin,HIGH);
  
  // For each full rotation of the pitch motor run the yaw motor one step
  for(int yy = 0; yy < myYawSteps; yy++) { 
    // Run the Pitch motor forward 1 partial rotation
    for(int x = 0; x < myPitchSteps; x++) {
      StepMotorForward(1);
      //delay should be included in rise/fall delay
    }
  
    // Run the Yaw motor forward 1 step since the pitch has run a full rotation
      StepMotorForward(2);
      //delay should be included in rice/fall delay
  }

  //Put the motors back to sleep to save power
  digitalWrite(slpPin,LOW);
}

