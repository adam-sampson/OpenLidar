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
#include <SoftwareSerial.h>

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

SoftwareSerial sf30_serial(serial_Rxd, serial_Txd);

// define variables to read into from SF30 laser distance
float sfDistance;
int Byte_L, Byte_H;

// define an array of characters to read serial into for bluetooth commands
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
const int riseDelay1 = 2;
const int fallDelay1 = 2;
const int riseDelay2 = 2; 
const int fallDelay2 = 2;
const unsigned long pitchMotDelay = 300;
const unsigned long yawMotDelay = 2400; 
unsigned long pitchMotMicroCounter; 
unsigned long yawMotMicroCounter; 

int myYawSteps = 400*8;  // This is a 400 step motor with Eighth Steps
int myPitchSteps = 400*8;  // This is a 400 step motor with Eight Steps

// define SD card variables
File myFile;
boolean mySdStatus = 0;
const int slaveSelect = 10; // Pin 10 is the chip select pin

/*
* Setup Function - Run once
*/
void setup() {
  // put your setup code here, to run once:
  // Set serial for bluetooth (or FTDI)
  Serial.begin(TerminalBAUD);
  //while (!Serial);  //Wait for serial before doing anything else

  //Setup the SF30
  sf30_serial.begin(sf30BAUD);
  //By Default we turn off the SF30 so it isn't running while we are in standby
  sf30_serial.print("#N");
  DEBUG_PRINTLN("Turned off SF30");
  sf30_serial.print("#R3:");  //Set the measurement resolution to 0.03m NOT smoothed
  sf30_serial.print("#U1665:");  //Set the measurement rate to 1665 per second
  sf30_serial.print("#p0:");  //Set the output to distance in meters.
  DEBUG_PRINT("SF set to 0.03m ");
  DEBUG_PRINT("resol. (unsmooth) ");
  DEBUG_PRINTLN("1665 times/second");

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
  DEBUG_PRINTLN("Initilazing SD card...");
  if (!SD.begin(4)) {
    DEBUG_PRINTLN("Initialization Failed.");
    mySdStatus = 0;
  }
  else {
    DEBUG_PRINTLN("Init. Complete.");
    mySdStatus = 1; 
  }
  if (mySdStatus == 1) {
    if (SD.exists("DEBUG.txt")) {
      DEBUG_PRINTLN("DEBUG.txt exists. Able to read/write SD card.");
      delay(1); //let bluetooth catch up
      /*DEBUG_PRINTLN("Deleting file.");
      SD.remove("DEBUG.txt");
      if (SD.exists("DEBUG.txt")) {
        DEBUG_PRINTLN("Delete failed.");
      }
      else {
        DEBUG_PRINTLN("Delete succeeded.");
      }
      */
    }
  else {
      DEBUG_PRINTLN("Creating fresh DEBUG.txt");
      myFile = SD.open("DEBUG.txt", FILE_WRITE);
      myFile.close();
  
      if (SD.exists("DEBUG.txt")) {
        DEBUG_PRINTLN("DEBUG.txt created. Able to read/write SD card.");
        delay(1); //let bluetooth catch up
      }
    }
  }
  
}

/*
* Loop Function - Run forever
*/
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
  if(serialData.startsWith("TestMotors")) {
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
  else if (serialData.startsWith("ScanRoom")) {
    delay(1);
    scanRoom(); 
    //Unset the command string so we don't repeat next loop
    serialData = ""; 
  }
  else if (serialData.startsWith("TestSF30")) {
    TestSF30();
    //Unset the command string so we don't repeat next loop
    serialData = "";
  }
}

/*
* readHall function - return 0 or 1 depending on magnet presence
*/
boolean readHall() {
  boolean digHallState = 0;
  return digitalRead(digHall);
  DEBUG_PRINT("Digital Hall State: ");
  DEBUG_PRINTLN(digHallState);
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

/*
* TestWriteToSD function - debugging purposes to see if SD works
*/
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

/*
* TestSF30 Function - See if we can read from the SF30 sensor
* Read the sensor 10 times and report over serial debug
*/
void TestSF30() {
  int isSfData = 0; 

  // Start the SF30
  sf30_serial.print("#Y");
  DEBUG_PRINTLN("Turned on SF30");
  
  //clear the buffer to avoid earlier readings
  while (sf30_serial.available() > 0) {
    Byte_H = sf30_serial.read();
    while (!sf30_serial.available()); //Don't get off order, wait for second bit.
    Byte_L = sf30_serial.read();
    //do nothing with this data, but throw it away as old data
  }
  DEBUG_PRINTLN("Cleared old data");
  delayMicroseconds(500);

  DEBUG_PRINTLN("Beginning SF30 Reading");
  delay(1); //let bluetooth catch up
  sf30_serial.available();
  for(int tt = 0; tt < 10; tt++) {
    if (sf30_serial.available() > 0) {
      while (!sf30_serial.available());
      Byte_H = sf30_serial.read();
      while (!sf30_serial.available()); //Don't get off order, wait for second bit.
      Byte_L = sf30_serial.read();
      sfDistance = (float(Byte_L))/256 + Byte_H;
      DEBUG_PRINT("Distance in meters = ");
      DEBUG_PRINTLN(sfDistance);
      isSfData += 1; 
    }
    delayMicroseconds(500); //delay 700us before next reading
  }
  if (isSfData == 0) {
    DEBUG_PRINTLN("No data collected from SF30");
    delay(1); //let bluetooth catch up
  }
  sf30_serial.print("#N");
  DEBUG_PRINT("Turned off SF30");
}

/*
* warmupSF30 function - warm up the sensor to get more precise readings
* run 30 seconds to warm up
*/
void warmupSF30 () {
  //Warm up the sensor by running it for 30 seconds.
  unsigned long warmupCounter;
  
  // Start the SF30
  sf30_serial.print("#Y");
  warmupCounter = millis();
  DEBUG_PRINTLN("Turned on SF30");

  DEBUG_PRINTLN("Begin SF30 Reading");
  sf30_serial.available();
  while (millis() - warmupCounter < 30000) {
    if (sf30_serial.available() > 0) {
      while (!sf30_serial.available());
      Byte_H = sf30_serial.read();
      while (!sf30_serial.available()); //Don't get off order, wait for second bit.
      Byte_L = sf30_serial.read();
      sfDistance = (float(Byte_L))/256 + Byte_H;
      //DEBUG_PRINT("Distance in meters = ");
      //DEBUG_PRINTLN(sfDistance); 
    }
  }
  sf30_serial.print("#N");
  
  delay(1); //let serial catch up
  DEBUG_PRINT("Warmed up");
}

/*
* scanRoom function - what we've been waiting for...actually scan the room
*/
void scanRoom() {
  DEBUG_PRINTLN("Starting room scan");
  //delay(1); //let bluetooth catch up
  
  //Make sure the motors are not sleeping
  digitalWrite(slpPin,HIGH);
  float range;
  float azimuth;
  float polar;
  float degPerYawStep = float(360)/myYawSteps;
  float degPerPitchStep = float(360)/myPitchSteps;
  String scanFilename;

  DEBUG_PRINTLN("Start SF30 Warmup");
  //delay(1); //let bluetooth catch up
  warmupSF30();

  DEBUG_PRINTLN("Start SD card");
  //delay(1); //let bluetooth catch up
  // Write the headers in our SD file
  scanFilename = createNewFileName();
  if (!SD.begin(4)) {
    DEBUG_PRINTLN("SD init. Failed.");
    return; //If we can't use SD card then quit.
  }

  myFile = SD.open(scanFilename, FILE_WRITE);
  myFile.print("Range,Azimuth,Polar");
  myFile.close();

  DEBUG_PRINTLN("Ready to scan");
  //delay(1); //let bluetooth catch up
  
  // For each full rotation of the pitch motor run the yaw motor one step
  for(int yy = 0; yy < myYawSteps; yy++) { 
    // Open the SD card to write to
    // open the sd card every yaw loop and close at end to make sure to save data
    myFile = SD.open(scanFilename, FILE_WRITE);
        
    if (!SD.exists("scanFilename")) {
     DEBUG_PRINTLN("Can access SD card, but can't open new file");
     delay(1); //let bluetooth catch up
     return;
    }

    // Calculate the azimuth for this loop
    azimuth = float(yy)*degPerYawStep;
    
    // Clear the SF30 memory in case delays have caused a queue
    while (sf30_serial.available() > 0) {
      Byte_H = sf30_serial.read();
      while (!sf30_serial.available()); //Don't get off order, wait for second bit.
      Byte_L = sf30_serial.read();
      //do nothing with this data, but throw it away as old data
    }
    //DEBUG_PRINTLN("Cleared old data");
  
    // Run the Pitch motor forward 1 rotation
    for(int x = 0; x < myPitchSteps; x++) {
      // Take a measurement
      while (sf30_serial.available() > 2) { //If there are more than 1 measurement to be read
        Byte_H = sf30_serial.read();
        while (!sf30_serial.available()); //Don't get off order, wait for second bit.
        Byte_L = sf30_serial.read();
        //do nothing with this data, but throw it away as old data
      }
      
      while (!sf30_serial.available()); //wait for next bit to be available
      Byte_H = sf30_serial.read();
      while (!sf30_serial.available()); //Don't get off order, wait for second bit.
      Byte_L = sf30_serial.read();
      range = (float(Byte_L))/256 + Byte_H;
      polar = float(yy)*degPerPitchStep;
      
      //Then move the motor forward 1 immediately after measurement
      //For now, run forward 2 eighth steps to get quarter step...need improvement
      StepMotorForward(1);
      StepMotorForward(1);

      // Write to SD card after moving motor to allow settling and measurement time
      myFile.print(range);
      myFile.print(",");
      myFile.print(azimuth);
      myFile.print(",");
      myFile.println(polar);
    }
  
    // Run the Yaw motor forward 1 step since the pitch has run a full rotation
    //For now, run forward 2 eighth steps to get quarter step...need improvement
      StepMotorForward(2);
      StepMotorForward(2);

    // Close the file to save it
      myFile.close();  

    // Check for a cancel command
    while(Serial.available()) {
    serialData = Serial.readString(); //read incoming data as a string
    DEBUG_PRINT("OL recieved: ");
    DEBUG_PRINTLN(serialData);
    }

    //Decide what to do with the command recieved
    if(serialData.startsWith("Cancel")) {
      DEBUG_PRINTLN("Cancel received.");
      return;
    }
        
  }

  //Put the motors back to sleep to save power
  digitalWrite(slpPin,LOW);
}

/*
* createNewFile function - as name says
* Check to see if file is taken and increment until unique name is found
* Return file name for use.
*/
String createNewFileName() {
  //This is a quick and dirty way to make sure a new file is created.
  //In the future ROM will be used to track file numbers.
  
  String baseFilename = "OL";
  String tempFilename;

  for (int fileNumber = 1; fileNumber < 9999; fileNumber++) {
    tempFilename = baseFilename + fileNumber + ".csv";
    if (!SD.exists(tempFilename)) {  //if the file doesn't exist, return the name
      DEBUG_PRINT("Found new filename: ");
      DEBUG_PRINTLN(tempFilename);
      return tempFilename;
    }
  }
  DEBUG_PRINT("Found files OL1 "); 
  DEBUG_PRINT("to OL9999. "); 
  DEBUG_PRINTLN("Clear SD card."); 
}

