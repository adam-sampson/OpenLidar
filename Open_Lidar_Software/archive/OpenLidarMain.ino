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
//#include <MemoryFree.h>

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
const unsigned long PROGMEM warmupMillis = 10000;

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
const unsigned long PROGMEM pitchMotDelay = 300;
const unsigned long PROGMEM yawMotDelay = 2400; 
unsigned long  pitchMotMicroCounter; 
unsigned long yawMotMicroCounter; 

int myYawSteps = 400*4;  // This is a 400 step motor with two Eighth Steps = quarter step
int myPitchSteps = 400*4;  // This is a 400 step motor with two Eighth Steps = quarter step

// define SD card variables
File myFile;
//boolean mySdStatus = 0;
const int PROGMEM slaveSelect = 10; // Pin 10 is the chip select pin
//const int PROGMEM readingsPerWrite = 320;

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
  //DEBUG_PRINTLN("Turned off SF30");
  sf30_serial.print(F("#R3:"));  //Set the measurement resolution to 0.03m NOT smoothed
  sf30_serial.print(F("#U1665:"));  //Set the measurement rate to 1665 per second
  sf30_serial.print(F("#p0:"));  //Set the output to distance in meters.
  //DEBUG_PRINT("SF set to 0.03m ");
  //DEBUG_PRINT("resol. (unsmooth) ");
  //DEBUG_PRINTLN("1665 times/second");

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
    Serial.println(F("SD Init Failed."));
    //mySdStatus = 0;
  }
  else {
    Serial.println(F("SD Init. Complete."));
    //mySdStatus = 1; 
  }
  /*
  if (mySdStatus == 1) {
    if (SD.exists("DEBUG.txt")) {
      DEBUG_PRINTLN("DEBUG.txt exists. Able to read/write SD card.");
      delay(100); //let bluetooth catch up
      DEBUG_PRINTLN("Deleting file.");
      SD.remove("DEBUG.txt");
      if (SD.exists("DEBUG.txt")) {
        DEBUG_PRINTLN("Delete failed.");
      }
      else {
        DEBUG_PRINTLN("Delete succeeded.");
      }
      
    }
  else {
      //DEBUG_PRINTLN("Creating fresh DEBUG.txt");
      myFile = SD.open("DEBUG.txt", FILE_WRITE);
      myFile.close();
  
      if (SD.exists("DEBUG.txt")) {
        //DEBUG_PRINTLN("DEBUG.txt created. Able to read/write SD card.");
        delay(1); //let bluetooth catch up
      }
    }
  }
  */
  /*
  if (SD.exists(F("DEBUG.txt"))) {  //if the file doesn't exist, return the name
      DEBUG_PRINTLN(F("Found Debug.txt"));
      delay(100);
    }
  */
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
    // Test motor 1 for 20 steps at default partial steps
    TestMotor(1,myPitchSteps);
    delay(500);
    //Test motor 2 for 20 steps at default partial steps
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
  else if (serialData.startsWith(F("TestSF30"))) {
    TestSF30();
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
    //Unset the command string so we don't repeat next loop
    serialData = "";
  }
}

/*
* readHall function - return 0 or 1 depending on magnet presence
*/
boolean readHall() {
  //boolean digHallState = 0;
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

/*
* TestWriteToSD function - debugging purposes to see if SD works
*/
/*
void TestWriteToSD() {
  //DEBUG_PRINT("SD Status: ");
  //DEBUG_PRINT(mySdStatus);
  //DEBUG_PRINT("  ");
  
  // re-open the file for writing
  myFile = SD.open("DEBUG.txt", FILE_WRITE);

  // if the file opened okay, write to it
  if (myFile) {
    //DEBUG_PRINTLN("Printing to file");
    myFile.println(F("File Found"));
    //myFile.println(mySdStatus);
    myFile.close(); // Make sure to close the file to save changes.
  }
  else {
    DEBUG_PRINTLN(F("Error test SD"));
  }
  DEBUG_PRINTLN(F("SD pass"));
}
*/

/*
* TestSF30 Function - See if we can read from the SF30 sensor
* Read the sensor 10 times and report over serial debug
*/
void TestSF30() {
  int isSfData = 0; 

  // Start the SF30
  sf30_serial.print("#Y");
  //DEBUG_PRINTLN("Turned on SF30");
  
  //clear the buffer to avoid earlier readings
  while (sf30_serial.available() > 0) {
    Byte_H = sf30_serial.read();
    while (!sf30_serial.available()); //Don't get off order, wait for second bit.
    Byte_L = sf30_serial.read();
    //do nothing with this data, but throw it away as old data
  }
  //DEBUG_PRINTLN("Cleared old data");
  delayMicroseconds(500);

  //DEBUG_PRINTLN("Beginning SF30 Reading");
  delay(1); //let bluetooth catch up
  sf30_serial.available();
  for(int tt = 0; tt < 10; tt++) {
    if (sf30_serial.available() > 0) {
      while (!sf30_serial.available());
      Byte_H = sf30_serial.read();
      while (!sf30_serial.available()); //Don't get off order, wait for second bit.
      Byte_L = sf30_serial.read();
      sfDistance = (float(Byte_L))/256 + Byte_H;
      DEBUG_PRINT(F("Distance in meters = "));
      DEBUG_PRINTLN(sfDistance);
      isSfData += 1; 
    }
    delayMicroseconds(500); //delay 700us before next reading
  }
  if (isSfData == 0) {
    DEBUG_PRINTLN(F("No data collected from SF30"));
    delay(1); //let bluetooth catch up
  }
  sf30_serial.print("#N");
  //DEBUG_PRINT("Turned off SF30");
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
  DEBUG_PRINTLN(F("Start Warmup"));

  //DEBUG_PRINTLN("Begin SF30 Reading");
  sf30_serial.available();
  while (millis() - warmupCounter < warmupMillis) {
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
  delay(100);
  //DEBUG_PRINTLN("Turned off SF30");
  
  DEBUG_PRINTLN(F("Warmed up"));
  delay(100); //let serial catch up
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

  char readingBuffer [9]; //Need 8 + 1 for trailing null from dtostrf
  char sdBuffer [512];  //Need to send 512 bytes per block to SD card

  //test varialbes to delete later
  float testrange = 39.999;
  float testazimuth = 99.9999;
  float testpolar = 100.99;
  float testintensity = 256.0;

  //Warmup the SF30
  warmupSF30();

  //Make sure the motors are not sleeping
  digitalWrite(slpPin,HIGH);

  //Open the SD card
  DEBUG_PRINTLN(F("Start SD card"));
  delay(10);

  //For now, overwrite old file...need file increment function
  if (SD.exists("OL1.CSV")) {
   //DEBUG_PRINTLN("Can access SD card, but can't open new file");
   //delay(1); //let bluetooth catch up
   SD.remove("OL1.CSV");
  }

  delay(100);

  myFile = SD.open("OL1.CSV", FILE_WRITE);
  myFile.println("Range,Azimuth,Polar");
  myFile.close();

  //test the file for debug purposes
  /*
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
  */

  //Begin scanning procedure
  //Turn the pitch motor 1 full turn while taking measurements
  //Write to SD card every 512 Bytes
  //Turn azimuth motor 1 'step' for each pitch motor turn
  
  //Turn on the SF30
  sf30_serial.print("#Y");
  delay(10);
  
  myFile = SD.open("OL1.CSV", FILE_WRITE);
  //myFile.println("Range,Azimuth,Polar,Intensity");
  //myFile.close();
  DEBUG_PRINTLN(F("Scanning"));
  delay(100); //let bluetooth catch up
  
  //Turn Azimuth
  //for(int aloop = 0; aloop < myYawSteps; aloop++) {
  for(int aloop = 0; aloop < 2; aloop++) {
    // Calculate the azimuth for this loop
    azimuth = float(aloop)*degPerYawStep;

    //Turn Pitch
    for(int ploop = 0; ploop < myPitchSteps; ploop++) {
    //for(int ploop = 0; ploop < 16; ploop++) {
      // Take a measurement
      //If there are more than 1 reading (2 bytes) then we have gotten ahead of ourselves
      while (sf30_serial.available() > 2) { //If there are more than 1 measurement to be read
        Byte_H = sf30_serial.read();
        while (!sf30_serial.available()); //Don't get off order, wait for second bit.
        Byte_L = sf30_serial.read();
        //do nothing with this data, but throw it away as old data
      }
      
      while (!sf30_serial.available()); //Wait until data is available
      Byte_H = sf30_serial.read();
      while (!sf30_serial.available()); //Don't get off order, wait for second bit.
      Byte_L = sf30_serial.read();
      range = (float(Byte_L))/256 + Byte_H;
      polar = float(ploop)*degPerPitchStep;

      //add readings to sdBuffer in 32 byte increments
      //8 range 1 ',' 8 azimuth 1 ',' 8 polar 1 ',' 3 intensity 1 'lf' 1'cr'
      //DEBUG_PRINT(freeMemory());
      //DEBUG_PRINT("    ");
      //DEBUG_PRINT(shotCounter%16);
      //DEBUG_PRINT("    ");
      //delay(100);
      dtostrf(range,8,4,readingBuffer);
      //dtostrf(testrange,8,4,readingBuffer);
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
      
      //Buffer is 512 bytes which is 16 rounds of 32 bytes
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

    // Check for a Cancel command
    while(Serial.available()) {
    serialData = Serial.readString(); //read incoming data as a string
      if(serialData.startsWith(F("Cancel"))) {
        DEBUG_PRINTLN(F("Cancel received."));
        //Close the file
        myFile.close();
        //Put the SF30 back to sleep to save power
        sf30_serial.print("#N");
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
}


/*
* ArchivescanRoom function - what we've been waiting for...actually scan the room
*/
/*
void ArchivescanRoom() {
  DEBUG_PRINTLN(F("Start room scan"));
  //delay(1); //let bluetooth catch up
  
  //Make sure the motors are not sleeping
  digitalWrite(slpPin,HIGH);
  //char tempString[1600][30];  //1600 is quarter pitch steps
  //char rangeStr[9],azimuthStr[9],polarStr[9];
  unsigned int tempByte_H, tempByte_L;
  //int tempRange;
  unsigned int tempRange[readingsPerWrite];
  int countCycle = 0;
  //int countTemp;
  float range;
  float azimuth;
  float polar;
  //int polarTemp[readingsPerWrite];
  float degPerYawStep = float(360)/myYawSteps;
  float degPerPitchStep = float(360)/myPitchSteps;
  String scanFilename;

  //DEBUG_PRINTLN("Start SF30 Warmup");
  //delay(1); //let bluetooth catch up
  warmupSF30();

  DEBUG_PRINTLN(F("Start SD card"));
  delay(10);

  if (SD.exists("OL1.CSV")) {
   //DEBUG_PRINTLN("Can access SD card, but can't open new file");
   //delay(1); //let bluetooth catch up
   SD.remove("OL1.CSV");
  }

  myFile = SD.open("OL1.CSV", FILE_WRITE);
  myFile.println("Range,Azimuth,Polar");
  myFile.close();

  DEBUG_PRINTLN(F("Scanning"));
  delay(10); //let bluetooth catch up
  //Turn on the SF30
  sf30_serial.print("#Y");
  delay(10);
  
  // For each full rotation of the pitch motor run the yaw motor one step
  for(int yy = 0; yy < myYawSteps; yy++) { 
    // Open the SD card to write to
    // open the sd card every yaw loop and close at end to make sure to save data
    

    myFile = SD.open("OL1.CSV", FILE_WRITE);
    //myFile = SD.open("OL1.CSV", O_CREAT | O_WRITE);

    if (!SD.exists("OL1.CSV")) {
     //DEBUG_PRINTLN("Can access SD card, but can't open new file");
     //delay(1); //let bluetooth catch up
     return;
    }

    // Calculate the azimuth for this loop
    azimuth = float(yy)*degPerYawStep;
    //dtostrf(azimuth,8,3,azimuthStr);
    
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
      //If there are more than 1 reading (2 bytes) then we have gotten ahead of ourselves
      while (sf30_serial.available() > 2) { //If there are more than 1 measurement to be read
        Byte_H = sf30_serial.read();
        while (!sf30_serial.available()); //Don't get off order, wait for second bit.
        Byte_L = sf30_serial.read();
        //do nothing with this data, but throw it away as old data
      }
      
      while (!sf30_serial.available());
      tempByte_H = sf30_serial.read();
      while (!sf30_serial.available()); //Don't get off order, wait for second bit.
      tempByte_L = sf30_serial.read();
      tempRange[x % readingsPerWrite] = tempByte_H * 256 + tempByte_L;
      //range = (float(Byte_L))/256 + Byte_H;
      //dtostrf(range,8,3,rangeStr);

      //polarTemp[x % readingsPerWrite] = x;
      //polar = float(x)*degPerPitchStep;
      //dtostrf(polar,8,2,polarStr);
      
      //Then move the motor forward 1 immediately after measurement
      //For now, run forward 2 eighth steps to get quarter step...need improvement
      StepMotorForward(1);
      StepMotorForward(1);

      if (myFile) {
      //Only write to the SD card every readingsPerWrite cycles
      if (x % readingsPerWrite == 0) {
        for (int n=readingsPerWrite*countCycle; n < readingsPerWrite*(countCycle+1); n++) {
          range = (float(tempRange[n%readingsPerWrite]))/256;
          //range = (float(tempRange[x % 64]))/256;
          polar = float(n)*degPerPitchStep;
          //polar = (float(polarTemp[x % 64]))*degPerPitchStep;
          //dtostrf(polar,8,2,polarStr)
          myFile.print(range);
          myFile.print(",");
          myFile.print(azimuth);
          myFile.print(",");
          myFile.println(polar);
          DEBUG_PRINTLN(range);
        }
        countCycle += 1;
      }
      }

    }
  
    // Run the Yaw motor forward 1 step since the pitch has run a full rotation
    //For now, run forward 2 eighth steps to get quarter step...need improvement
      StepMotorForward(2);
      StepMotorForward(2);

      countCycle = 0;

    //for (int n=0; n < myPitchSteps; n++) {
    //  range = (float(tempRange[n]))/256;
    //  myFile.print(range);
    //  myFile.print(",");
    //  myFile.print(azimuth);
    //  myFile.print(",");
    //  myFile.println(n);
    //}

    // save the file in case of crash
      myFile.close();  

    // Check for a cancel command
    while(Serial.available()) {
    serialData = Serial.readString(); //read incoming data as a string
    //DEBUG_PRINT("OL recieved: ");
    DEBUG_PRINTLN(serialData);
    }

    //Decide what to do with the command recieved
    if(serialData.startsWith(F("Cancel"))) {
      DEBUG_PRINTLN(F("Cancel received."));
      //Put the motors back to sleep to save power
      sf30_serial.print("#N");
      delay(10);
      //DEBUG_PRINTLN("Turned off SF30");
      digitalWrite(slpPin,LOW);
      return;
    }
      
  }

  sf30_serial.print("#N");
  delay(10);
  //DEBUG_PRINTLN("Turned off SF30");
  //Put the motors back to sleep to save power
  digitalWrite(slpPin,LOW);
}
*/

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
