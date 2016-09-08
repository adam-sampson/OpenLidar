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
 *       D0 - Pin A3
 *     
 *     Motor 1 connected to digital output pins
 *       Step - Pin 3
 *       Direction - Pin 4
 *     
 *     Motor 2 connected to digital output pins
 *       Step - Pin 5
 *       Direction - Pin 6
 *     
 *     SF30-B connected to digital input/output pins through SoftwareSerial
 *       ? - Pin A0 as Input
 *       ? - Pin A1 as Ouptput
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

#define BAUD (9600)

// define pins numbers
const int stepPin1 = 3; // motor 1 step pin
const int dirPin1 = 4;  // motor 1 direction pin
const int stepPin2 = 5; // motor 2 step pin 
const int dirPin2 = 6;  // motor 2 direction pin
const int digHall = A3; // digital hall effect pin
//const int anaHall = A2; // analog hall effect pin

// define motor profile
const int riseDelay = 250;
const int fallDelay = 250;
int myYawSteps = 400;  // This is a 400 step motor
int myPitchSteps = 400 * 4;  // This is a 400 step motor on quarter step mode

// define SD card variables
File myFile;
boolean mySdStatus = 0;
const int slaveSelect = 10; // Pin 10 is the chip select pin

void setup() {
  // put your setup code here, to run once:
  // Set serial for bluetooth
  Serial.begin(BAUD);

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

void loop() {
  // Set variables
  boolean digHallState = 0;
  //int anHallState = 0;
  
  unsigned long microcounter;
  microcounter = micros();

  delay(10000); // 10 second delay to get hands out of the way

  // Collect the hall effect state
  digHallState = digitalRead(digHall);
  //anHallState = analogRead(anaHall);

// For each full rotation of the pitch motor run the yaw motor one step
/*
for(int yy = 0; yy < myYawSteps; yy++) { 
    // Run the Pitch motor forward 1 partial rotation
    for(int x = 0; x < myPitchSteps; x++) {
      StepMotorForward(stepPin1,dirPin1);
      //delay(1);
      delayMicroseconds(250);
    }
  
    // Run the Yaw motor forward 1 step
      StepMotorForward(stepPin2,dirPin2);
      delayMicroseconds(250);
    }
*/

  DEBUG_PRINT("SD Status: ");
  DEBUG_PRINT(mySdStatus);
  DEBUG_PRINT("  ");
  DEBUG_PRINT("Digital Hall State: ");
  DEBUG_PRINTLN(digHallState);
  //DEBUG_PRINT(" , ");
  //DEBUG_PRINTLN(anHallState);
  
  // re-open the file for writing
  myFile = SD.open("DEBUG.txt", FILE_WRITE);

  // if the file opened okay, write to it
  if (myFile) {
    DEBUG_PRINTLN("Printing to file");
    myFile.print("SD Status: ");
    myFile.print(mySdStatus);
    myFile.print("  ");
    myFile.print("Digital Hall State: ");
    myFile.println(digHallState);
    myFile.close(); // Make sure to close the file to save changes.
  }
  else {
    DEBUG_PRINTLN("Error opening file to write.");
  }
}

void StepMotorForward(int myStepPin, int myDirPin){
  digitalWrite(myDirPin,HIGH);
  digitalWrite(myStepPin,HIGH);
  delayMicroseconds(riseDelay);
  digitalWrite(myStepPin,LOW);
  delayMicroseconds(fallDelay);
}

