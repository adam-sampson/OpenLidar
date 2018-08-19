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
 *       Question: Is there a 20byte transmit limit?
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
 *     Motor 1 (pitch) connected to digital output pins
 *       Step - Pin 4
 *       Direction - Pin 3
 *     
 *     Motor 2 (yaw) connected to digital output pins
 *       Step - Pin 6
 *       Direction - Pin 5
 *     
 *     SF30-B connected to digital input/output pins through SoftwareSerial
 *       Rx Arduino - TXD SF30 (yellow) - Pin A0 as INPUT
 *       Tx Arduino - RXD SF30 (orange) - Pin A1 as OUTPUT
 */

#include <SPI.h>
#include <SD.h>
//#include <SoftwareSerial.h>
//#include <MemoryFree.h>
#include <Wire.h>
#include <LIDARLite.h>

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINTLN(x)  Serial.println (x)
 #define DEBUG_PRINT(x) Serial.print (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

#define TerminalBAUD 9600  //38400    //Baud rate of serial or bluetooth output
#define serial_Rxd A0         //Pin definitions to match wiring to SF30
#define serial_Txd A1
#define sf30BAUD 38400        //SF30 baud rate

//SoftwareSerial sf30_serial(serial_Rxd, serial_Txd);

// define variables to read into from SF30 laser distance
//float sfDistance;
//int Byte_L, Byte_H;

// define variables for Lidar Lite 
LIDARLite myLidarLite;
int llDistance = 0;

// define an array of characters to read serial into for bluetooth commands
String serialData;

// define pins numbers
const int PROGMEM slpPin = 8;
const int PROGMEM MS1Pin = 9;
const int PROGMEM MS2Pin = A3;

const int PROGMEM stepPin1 = 4; // motor 1 step pin
const int PROGMEM dirPin1 = 3;  // motor 1 direction pin
const int PROGMEM stepPin2 = 6; // motor 2 step pin 
const int PROGMEM dirPin2 = 5;  // motor 2 direction pin
const int PROGMEM digHall = A2; // digital hall effect pin

// define motor profile
const int PROGMEM riseDelay1 = 2;
const int PROGMEM fallDelay1 = 2;
const int PROGMEM riseDelay2 = 2; 
const int PROGMEM fallDelay2 = 2;
const unsigned long PROGMEM pitchMotDelay = 350;
const unsigned long PROGMEM yawMotDelay = 2400; 
unsigned long  pitchMotMicroCounter; 
unsigned long yawMotMicroCounter; 

int myYawSteps = 400*2;  // This is a 400 step motor with Eighth Steps and yaw only goes half around
int myPitchSteps = 400*4;  // This is a 400 step motor with Eight Steps

// define SD card variables
File myFile;
//boolean mySdStatus = 0;
const int PROGMEM slaveSelect = 10; // Pin 10 is the chip select pin
const int PROGMEM readingsPerWrite = 320;

/*
* Setup Function - Run once
*/
void setup() {
  // put your setup code here, to run once:
  // Set serial for bluetooth (or FTDI)
  Serial.begin(TerminalBAUD);
  //while (!Serial);  //Wait for serial before doing anything else

  // Set up lidar lite
  // acquisition count down to 1/3 and get more noise for more speed (option 1)
  // enable 400kz i2c (option true)
  myLidarLite.begin(1,true);
  //  Next we need to take 1 reading with preamp stabilization and reference pulse (these default to true)
  Serial.println(myLidarLite.distance());

  // Set motors to sleep mode to conserve power
  pinMode(slpPin,OUTPUT);
  digitalWrite(slpPin,LOW);

  // Set motor delay variables
  pitchMotMicroCounter = micros();
  yawMotMicroCounter = micros (); 

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

  // Sets the Hall Effect pin as Input
  pinMode(digHall,INPUT);

 
  // Set up the SD card
  pinMode(slaveSelect,OUTPUT); //attempting to prevent hardware from setting Arduino as Slave
  //DEBUG_PRINTLN(F("Initilazing SD card..."));
  if (!SD.begin(slaveSelect)) {
    DEBUG_PRINTLN(F("SD Init Failed."));
    //mySdStatus = 0;
  }
  else {
    DEBUG_PRINTLN(F("SD Init. Complete."));
    //mySdStatus = 1; 
  }
  DEBUG_PRINTLN(F("Setup complete"));
}

/*
* Loop Function - Run forever
*/
void loop() {
  // Set variables
  unsigned long microcounter;
  microcounter = micros();
  String testFilename;

  //delay(10000); // 10 second delay to get hands out of the way

  //Listen for bluetooth commands
  while(Serial.available()) {
    serialData = Serial.readString(); //read incoming data as a string
    //DEBUG_PRINT("OL recieved: ");
    DEBUG_PRINTLN(serialData);
  }

  //Decide what to do with the command recieved
  if(serialData.startsWith(F("TestMotors"))) {
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
  else if (serialData.startsWith(F("ScanRoom"))) {
    delay(1);
    scanRoom(); 
    //Unset the command string so we don't repeat next loop
    serialData = ""; 
  }
  else if (serialData.startsWith(F("TestSD"))) {
    if (SD.exists("DEBUG.txt")) {  //if the file doesn't exist, return the name
      DEBUG_PRINTLN(F("Found DEBUG.txt"));
      delay(100);
    }
    testFilename = createNewFileName();
    DEBUG_PRINT(F("Created file: "));
    DEBUG_PRINTLN(testFilename);
    serialData = "";
  }
}

/*
* readHall function - return 0 or 1 depending on magnet presence
*/
boolean readHall() {
  boolean digHallState = 0;
  return digitalRead(digHall);
  //DEBUG_PRINT("Digital Hall State: ");
  //DEBUG_PRINTLN(digHallState);
}

/*
* StepMotorForward function - move selected motor forward 1 step
* Currently allows motor 1 or motor 2 as inputs only. 
* Motor movement delay is coded through micro counter
*/
void StepMotorForward(int Motor){
  //temp variables
  int tempStepPin;
  int tempDirPin;

  switch(Motor) {
    case 1:
      while (micros() - pitchMotMicroCounter < pitchMotDelay) {
        //wait until enough time has passed before trying to move the motor again without using delay
      }
      tempStepPin = stepPin1;
      tempDirPin = dirPin1;
      digitalWrite(tempDirPin,HIGH);
      digitalWrite(tempStepPin,HIGH);
      delayMicroseconds(riseDelay1);
      digitalWrite(tempStepPin,LOW);
      delayMicroseconds(fallDelay1);
      //reset the counter to new time
      pitchMotMicroCounter = micros();
      break;
    case 2:
      while (micros() - yawMotMicroCounter < yawMotDelay) {
        //wait until enough time has passed before trying to move the motor again without using delay
      }
      tempStepPin = stepPin2;
      tempDirPin = dirPin2;
      digitalWrite(tempDirPin,HIGH);
      digitalWrite(tempStepPin,HIGH);
      delayMicroseconds(riseDelay2);
      digitalWrite(tempStepPin,LOW);
      delayMicroseconds(fallDelay2);
      //reset the counter to new time
      yawMotMicroCounter = micros();
      break;
    default: //we only have two motors...cancel if something else selected
      return;
      break;
  }
  
  
/*
* TestMotor function - basic test that motors are working
*/
}

void TestMotor(int Motor, int Steps) {
  //Make sure the motors are not sleeping
  digitalWrite(slpPin,HIGH);

  //
  for (int i=0; i <= Steps; i++) {
    StepMotorForward(Motor);
  }
}

/*----------------------------------------------------------------
* scanRoom function - what we've been waiting for...actually scan the room
*/
void scanRoom() {
  DEBUG_PRINTLN(F("Start room scan"));
  
  //Define Variables
  float range,azimuth,polar;
  float degPerYawStep = float(360)/myYawSteps;
  float degPerPitchStep = float(360)/myPitchSteps;
  unsigned long shotCounter = 0;
  unsigned int bufferCounter = 0;
  int signalStrength = 0;

  char readingBuffer [9]; //Need 8 + 1 for trailing null from dtostrf
  char sdBuffer [512];  //Need to send 512 bytes per block to SD card

  //test varialbes to delete later
  //float testrange = 39.999;
  //float testazimuth = 99.9999;
  //float testpolar = 100.99;
  float testintensity = 0;

  //Make sure the motors are not sleeping
  digitalWrite(slpPin,HIGH);

  //Open the SD card
  DEBUG_PRINTLN(F("Start SD card"));
  delay(10);

  if (SD.exists("OL2.CSV")) {
   //DEBUG_PRINTLN("Can access SD card, but can't open new file");
   //delay(1); //let bluetooth catch up
   SD.remove("OL1.CSV");
  }

  delay(100);

  myFile = SD.open("OL2.CSV", FILE_WRITE);
  myFile.println("Range,Azimuth,Polar");
  myFile.close();

  //test the file for debug purposes
  delay(100);
  myFile = SD.open("OL1.CSV"); //Open for reading
  if(myFile) {
    Serial.println(F("OL1.CSV opened"));
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  }
  else {
    Serial.println(F("Error opening OL1.SCV"));
  }

  //Begin scanning procedure
  //Turn the pitch motor 1 full turn while taking measurements
  //Write to SD card every 512 Bytes
  //Turn azimuth motor 1 'step' for each pitch motor turn
  
  myFile = SD.open("OL1.CSV", FILE_WRITE);
  DEBUG_PRINTLN(F("Scanning"));
  delay(100); //let bluetooth catch up
  
  //Turn Azimuth
  //for(int aloop = 0; aloop < myYawSteps; aloop++) {
  for(int aloop = 0; aloop < myYawSteps; aloop++) {
    // Calculate the azimuth for this loop
    azimuth = float(aloop)*degPerYawStep;

    //  Every azimuth need to take 1 reading with preamp stabilization and reference pulse (these default to true)
    Serial.println(myLidarLite.distance());

    //Turn Pitch
    for(int ploop = 0; ploop < myPitchSteps; ploop++) {
    //for(int ploop = 0; ploop < 16; ploop++) {
      // Take a measurement without preamp stabilization and reference pulse
      llDistance = myLidarLite.distance(false,false);
      // Capture the intensity
      //signalStrength = myLidarLite.signalStrength();

      //To make this code match SF30 code...for now be lazy
      range = float(llDistance);
      testintensity = float(signalStrength);
      
      //add readings to sdBuffer in 32 byte increments
      //8 range 1 ',' 8 azimuth 1 ',' 8 polar 1 ',' 3 intensity 1 'lf' 1'cr'
      dtostrf(range,8,4,readingBuffer);
      //dtostrf(testrange,8,4,readingBuffer);
      //DEBUG_PRINT(freeMemory());
      //DEBUG_PRINT("    ");
      //DEBUG_PRINT(shotCounter%16);
      //DEBUG_PRINT("    ");
      //delay(100);
      for (int br= 0; br < 8; br++) {
        sdBuffer[(32*(shotCounter % 16)+br)] = readingBuffer[br];
      }
      sdBuffer[(32*(shotCounter%16)+8)] = ',';
      dtostrf(azimuth,8,4,readingBuffer);
      //dtostrf(testazimuth,8,4,readingBuffer);
      for (int ba= 0; ba < 8; ba++) {
        sdBuffer[(32*(shotCounter % 16)+9+ba)] = readingBuffer[ba];
      }
      sdBuffer[(32*(shotCounter%16)+17)] = ',';
      dtostrf(polar,8,4,readingBuffer);
      //dtostrf(testpolar,8,4,readingBuffer);
      for (int bp= 0; bp < 8; bp++) {
        sdBuffer[(32*(shotCounter % 16)+18+bp)] = readingBuffer[bp];
      }
      sdBuffer[(32*(shotCounter%16)+26)] = ',';
      dtostrf(testintensity,5,0,readingBuffer);
      for (int bi= 0; bi < 3; bi++) {
        sdBuffer[(32*(shotCounter % 16)+27+bi)] = readingBuffer[bi+2];
      }
      sdBuffer[(32*(shotCounter % 16)+30)] = ' ';
      sdBuffer[(32*(shotCounter % 16)+31)] = 10;
      

      //If we are at the end of the buffer shotcounter % 16 = 15
      //Send the buffer to the sd card
      if (shotCounter % 16 == 15) {
        //DEBUG_PRINTLN("Write Loop");
        //delay(100);
        myFile.write(sdBuffer,512);
      }

      //Step the pitch motor forward 1 step
      //For now, run forward 2 eighth steps to get quarter step...need improvement
      StepMotorForward(1);
      StepMotorForward(1);

      //Increment the shot counter every time we take a shot
      shotCounter += 1;
    } //end pitch loop

    //DEBUG_PRINTLN("Yaw Loop");
    //Once every yaw cycle save the file in case of crash
    myFile.flush();

    // Check for a cancel command
    while(Serial.available()) {
    serialData = Serial.readString(); //read incoming data as a string
      if(serialData.startsWith(F("Cancel"))) {
        DEBUG_PRINTLN(F("Cancel received."));
        //Close the file
        myFile.close();
        delay(10);
        //Turn off the motors
        digitalWrite(slpPin,LOW);
        return;
      }
    }

    // Run the Yaw motor forward 1 step since the pitch has run a full rotation
    //For now, run forward 2 eighth steps to get quarter step...need improvement
    StepMotorForward(2);
    StepMotorForward(2);
    
  }  //end azimuth loop

  //Make sure the motors are sleeping
  digitalWrite(slpPin,LOW);
  //myFile.println("Testafterloop");
  myFile.close();
  //test the file for debug purposes
  delay(100);
  /*
  myFile = SD.open("OL1.CSV"); //Open for reading
  if(myFile) {
    Serial.println(F("OL1.CSV opened"));
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  }
  else {
    Serial.println(F("Error opening OL1.SCV"));
  }
  */
  DEBUG_PRINTLN("Scan Complete");
}

/*
* createNewFile function - as name says
* Check to see if file is taken and increment until unique name is found
* Return file name for use.
*/
String createNewFileName() {
  //This is a quick and dirty way to make sure a new file is created.
  //In the future ROM will be used to track file numbers.
  DEBUG_PRINTLN(F("Create FN"));
  delay(100);
  String baseFilename = "OL";
  String tempFilename;
  char tempBuffer[] = "OL1.CSV";

  /*
  if (mySdStatus = 0) {
    DEBUG_PRINTLN(F("Initialization Failed."));
  }
  else {
    DEBUG_PRINTLN(F("SD was setup"));
  */
  
  for (int fileNumber = 1; fileNumber < 5; fileNumber++) {
    //DEBUG_PRINTLN("ForLoop");
    //delay(100);
    tempFilename = baseFilename + String(fileNumber, DEC) + ".csv";
    //tempFilename.toCharArray(tempBuffer,tempFilename.length()+1);
    DEBUG_PRINT(F("Temp file: "));
    DEBUG_PRINTLN(tempBuffer);
    delay(100);
    if (SD.exists(tempBuffer)) {  //if the file doesn't exist, return the name
      DEBUG_PRINT(F("Found new filename: "));
      delay(100);
      DEBUG_PRINTLN(tempFilename);
      return tempFilename;
    }
  //}
  }
  
  //return "OL0001.csv";
  DEBUG_PRINTLN(F("SD full")); 
  delay(100);
  return "";
}
