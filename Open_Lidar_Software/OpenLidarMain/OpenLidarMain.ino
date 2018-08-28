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
 *     Bluetooth Connected to the TX/RX pins
 *       TX - Pin TX5 on Arduino to RX on Bluetooth Module
 *       RX - Pin RX5 on Artuino to TX on Bluetooth Module
 *       Use Serial5.function to control this.
 *     
 *     Motor Details
 *          **************************
 *          *              Mode0 Mode1 Mode2 * 
 *          *Full Step     Low   Low   Low   *
 *          *1/2 Step      High  Low   Low   *
 *          *1/4 Step      Low   High  Low   *
 *          *1/8 Step      High  High  Low   *
 *          *1/16 Step     Low   Low   High  *
 *          *1/32 Step     High  High  High  *
 *          **************************
 *     
 *     Motor 1A (pitch) connected to digital output pins
 *       EN  - 2
 *       M0  - 3
 *       M1  - 4
 *       M2  - 5
 *       RST - 6
 *       SLP - 7
 *       STP - 8
 *       DIR - 9
 *     
 *     Motor 0.6A (yaw) connected to digital output pins
 *       EN  - 12
 *       M0  - 24
 *       M1  - 25
 *       M2  - 26
 *       RST - 27
 *       SLP - 28
 *       STP - 29
 *       DIR - 30
 *       
 *     Lidar Lite V3 connected to I2C pins
 *       SCL0 - digital pin 19
 *       SDA0 - digital pin 18
 *
 * Warning SdFatSdio and SdFatSdioEX normally should 
 * not both be used in a program.
 * Each has its own cache and member variables.
 * 
  */

// Define Libraries
#include <DRV8825.h>
// #include <Wire.h>
#include <i2c_t3.h>
#include <I2CFunctions.h> 
#include <LidarObject.h>
#include <LidarController.h>
#include "SdFat.h"

// Define whether or not to use DEBUG mode
#define DEBUG

// Define serial command object
#define mySerial Serial5 // Define to serial you want to use...Serial for usb, Serial5 for bluetooth
const byte numChars = 32;
char receivedChars[numChars];
boolean receiveFull = false;
#define MAX_WORD_COUNT 3
#define MIN_WORD_COUNT 2
char *Words[MAX_WORD_COUNT];
boolean lockSerial = false; // Some functions may want to allow only "Cancel" as a command
boolean cancel = false; // Functions that allow cancel will check for this flag each major loop

#ifdef DEBUG
 #define DEBUG_PRINTLN(x)  mySerial.println (x)
 #define DEBUG_PRINT(x) mySerial.print (x)
#else
 #define DEBUG_PRINT(x) // do nothing
 #define DEBUG_PRINTLN(x) // do nothing
#endif

#define TerminalBAUD 9600  //38400    //Baud rate of serial or bluetooth output

// Define Lidar related variables
#define WIRE400K true
// Trigger pin, can be unplugged
#define Z1_LASER_TRIG 0
// Enable pin, IMPORTANT
#define Z1_LASER_EN 17
// Mode pin, can be unplugged
#define Z1_LASER_PIN 0
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define Z1_LASER_AD 0x62

#define NUMBER_OF_LASERS 1

// Delays
long currMicro, lastMicro;
int shotMicroDelay = 100;
boolean distanceReady = false;
int microCnt;

// Create lasers
static LidarController Controller;
static LidarObject LZ1;

// Define motor structure
typedef struct {
  int EN;
  int M0;
  int M1;
  int M2;
  int RST;
  int SLP;
  int STP;
  int DIR;
  int steps; // Number of steps in a full turn
  int microstep; //Fraction to microstep. I.E. 4 is quarter step.
  int rpm; // Max speed of motor in rotations per minute
}motorStruct;

motorStruct pitch = {2,3,4,5,6,7,8,9,400,32,180};
motorStruct yaw = {12,24,25,26,27,28,29,30,8000,32,10};

// Initiate the motors using the DRV8825 library
DRV8825 pitchMot(pitch.steps,pitch.DIR,pitch.STP,pitch.EN,pitch.M0,pitch.M1,pitch.M2);
DRV8825 yawMot(yaw.steps,yaw.DIR,yaw.STP,yaw.EN,yaw.M0,yaw.M1,yaw.M2);

// Define variables for writing to the SD Card
//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))

// file system
SdFatSdio sd;
SdFile file;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "OL"
char fileName[10] = FILE_BASE_NAME "000.sph";

// define a struct to hold a lidar reading
typedef struct{
  uint16_t radius;
  float yaw;
  float pitch;
  byte intensity;
}Readings;

Readings reading;

/*
* Setup Function - Run once
*/
void setup() {
  // put your setup code here, to run once:
  
  // Set serial communication for whichever serial port chosen above
  mySerial.begin(TerminalBAUD);

  pitchMot.begin(pitch.rpm);
  yawMot.begin(yaw.rpm);
  
  pitchMot.disable(); // No need to use power until command received
  yawMot.disable(); // No need to use power until command received

  pinMode(pitch.SLP, OUTPUT);
  pinMode(pitch.RST, OUTPUT);
  pinMode(yaw.SLP,   OUTPUT);
  pinMode(yaw.RST,   OUTPUT);

  // Disable sleep and set reset. Rely on EN pin to enable or disable stepper.
  digitalWrite(pitch.SLP,LOW);
  digitalWrite(yaw.SLP,  LOW);
  digitalWrite(pitch.RST,LOW);
  digitalWrite(yaw.RST  ,LOW);  

  // Configure lasers
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_TRIG, Z1_LASER_AD, 2, DISTANCE, 'A');
  LZ1.setCallbackDistance(&distance_callback);
  // Add the laser to the Controller
  Controller.add(&LZ1, 0);
  
  delay(100);
  Controller.begin(WIRE400K);
  delay(100);

  //setup sd card
  if (!sd.begin()) {
    sd.initErrorHalt();
  }

  mySerial.println(F("OpenLidar Ready."));
}

/*
* Loop Function - Run forever
*/
void loop() {
  // Set variables

  //delay(5000); // 5 second delay to get hands out of the way
  checkSerial();
  cancel = false; // The loop doesn't care about cancel, so uncheck it
}

// Write data header.
void writeHeader() {
  file.print(F("radius(int),pitch(flt),yaw(flt),intensity(byte)"));
  file.println();
}

// Find a new file name
void findFileName(){
  // Find an unused file name.
  uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 2] != '9') {
      fileName[BASE_NAME_SIZE + 2]++;
    } else if (fileName[BASE_NAME_SIZE+1] != '9') {
      fileName[BASE_NAME_SIZE + 2] = '0';
      fileName[BASE_NAME_SIZE+1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
}

void checkSerial(){
  static byte ndx = 0;
  char endMarker = ';';
  char rc;
  
  while(mySerial.available()>0 && receiveFull == false){
    // DEBUG_PRINT("Receiving Data.");
    rc = mySerial.read();
    if (rc != endMarker){
      receivedChars[ndx] = rc;
      ndx++;
      if(ndx >= numChars) { //string not allowed to exceed numChars
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      receiveFull = true;
    }
  }

  if (receiveFull == true){
    parseCommand();
  }
}

void parseCommand(){
  DEBUG_PRINT(F("Parsed Command:"));
  receiveFull = false;
  byte word_count = 0;
  int temp;
  char * item = strtok (receivedChars, " ,"); //getting first word (uses space & comma as delimeter)
  
  while (item) {
    if (word_count >= MAX_WORD_COUNT) {
      break;
    }
    Words[word_count] = item;
    item = strtok (NULL, " ,"); //getting subsequence word
    word_count++;
  }
  // Clear words from previous loops
  while (word_count < MAX_WORD_COUNT) {
    item = "";
    Words[word_count] = item;
    word_count++;
  }

  #ifdef DEBUG
  if(Words[0]) {
    DEBUG_PRINT(Words[0]);
    DEBUG_PRINT(" ");
  }
  if(Words[1]) {
    DEBUG_PRINT(Words[1]);
    DEBUG_PRINT(" ");
  }
  if(Words[2]) {
    DEBUG_PRINT(Words[2]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN("...");
  #endif

  // If a function has locked serial it will only accept the "cancel" command
  if(lockSerial == true){
    if(strcmp(Words[0],"cancel") == 0){
      cancel = true;
      DEBUG_PRINTLN("Cancel received and flagged.");
    }
  }
  // If the serial isn't locked run any function requested
  else {
    if(strcmp(Words[0],"cancel") == 0){
      cancel = true;
      DEBUG_PRINTLN("Cancel received and flagged.");
    }
    else if(strcmp(Words[0],"pitch") == 0){
      DEBUG_PRINT("Pitch received. ");
      if(strcmp(Words[1],"turn") == 0){
        DEBUG_PRINT(Words[1]);
        if(Words[2]){
          temp = atoi(Words[2]);
          DEBUG_PRINTLN(temp);
          pitchMot.rotate(temp);
        }
      }
      else if(strcmp(Words[1],"rpm") == 0){
        if(Words[2]){
          temp = atoi(Words[2]);
          pitch.rpm = temp;
          pitchMot.setRPM(pitch.rpm);
        }
      }
      else if(strcmp(Words[1],"microstep") == 0){
        if(Words[2]){
          temp = atoi(Words[2]);
          pitch.microstep = temp;
          pitchMot.setMicrostep(pitch.microstep);
        }
      }
      else if(strcmp(Words[1],"enable") == 0){
        digitalWrite(pitch.SLP,HIGH);
        digitalWrite(pitch.RST,HIGH);
        pitchMot.enable();
      }
      else if(strcmp(Words[1],"disable") == 0){
        pitchMot.disable();
        digitalWrite(pitch.SLP,LOW);
        digitalWrite(pitch.RST,LOW);
      }
    }
    else if(strcmp(Words[0],"yaw") == 0){
      DEBUG_PRINTLN("Yaw received.");
      if(strcmp(Words[1],"turn") == 0){
        DEBUG_PRINT(Words[1]);
        if(Words[2]){
          temp = atoi(Words[2]);
          DEBUG_PRINTLN(temp);
          yawMot.rotate(temp);
        }
      }
      else if(strcmp(Words[1],"rpm") == 0){
        if(Words[2]){
          temp = atoi(Words[2]);
          yaw.rpm = temp;
          yawMot.setRPM(yaw.rpm);
        }
      }
      else if(strcmp(Words[1],"microstep") == 0){
        if(Words[2]){
          temp = atoi(Words[2]);
          yaw.microstep = temp;
          yawMot.setMicrostep(yaw.microstep);
        }
      }
      else if(strcmp(Words[1],"enable") == 0){
        digitalWrite(yaw.SLP,HIGH);
        digitalWrite(yaw.RST,HIGH);
        yawMot.enable();
      }
      else if(strcmp(Words[1],"disable") == 0){
        yawMot.disable();
        digitalWrite(yaw.SLP,LOW);
        digitalWrite(yaw.RST,LOW);
      }
    }
    else if(strcmp(Words[0],"lidar") == 0){
      DEBUG_PRINTLN("Lidar received.");
      if(strcmp(Words[1],"dist") == 0){
        DEBUG_PRINT(Words[1]);
        // Take a lidar distance
        Controller.spinOnce();
        delay(10);
        laserprint();
      }
      else if(strcmp(Words[1],"scan") == 0){
        DEBUG_PRINT(Words[1]);
        if(Words[2]){
          simpleScan(Words[2]);
        }
      }
      else if(strcmp(Words[1],"test") == 0){
        DEBUG_PRINT("Distance Ready? ");
        DEBUG_PRINTLN(distanceReady);
        DEBUG_PRINT(Words[1]);
        testLidarCallback();
      }
    }
    else if(strcmp(Words[0],"bluetooth") == 0){
      DEBUG_PRINTLN("Bluetooth received.");
    }
    else if(strcmp(Words[0],"sd") == 0){
      DEBUG_PRINTLN("SD received.");
      if(strcmp(Words[1],"test")==0){
        DEBUG_PRINT(Words[1]);
        if(Words[2]){
          testSD(Words[2]);
        } else {
          Words[2] = "Test";
          testSD(Words[2]);
        }
      }
    }
    else if(strcmp(Words[0],"motor") == 0){
      DEBUG_PRINTLN("Motor received.");
      if(strcmp(Words[1],"enable")==0) enableMotors();
      else if(strcmp(Words[1],"disable")==0) disableMotors();
    }
  }
}

void testLidarCallback(){
  microCnt = micros();
  Controller.spinOnce();
}

void laserprint(){
  mySerial.print(" Measure: ");
  mySerial.print(LZ1.distance);
  mySerial.print(" Signal strength: ");
  mySerial.println(LZ1.strength);
  //mySerial.print(" Velocity: ");
  //mySerial.println(LZ1.velocity);
}

void clearSerial(){
  while(mySerial.available() > 0){
    mySerial.read();
  }
}

void enableMotors() {
  digitalWrite(pitch.SLP,HIGH);
  digitalWrite(pitch.RST,HIGH);
  pitchMot.enable();
  digitalWrite(yaw.SLP,  HIGH);
  digitalWrite(yaw.RST  ,HIGH);  
  yawMot.enable();
}

void disableMotors() {
  pitchMot.disable();
  digitalWrite(pitch.SLP,LOW);
  digitalWrite(pitch.RST,LOW);
  yawMot.disable();
  digitalWrite(yaw.SLP,  LOW);
  digitalWrite(yaw.RST  ,LOW);  
}

void testSD(char* inText){
  // Open the file for writing
  char testFile[10] = "test.txt";
  if (!file.open(testFile, O_CREAT | O_WRITE | O_EXCL)) {
    DEBUG_PRINTLN("Error Opening File");
    error("file.open");
    return;
  }
  
  // Write a header
  writeHeader();

  file.print(inText);
  file.close();
}

void distance_callback(LidarObject* self){
  //int microDiff = micros() - microCnt;
   //DEBUG_PRINT("Dist: ");
   //DEBUG_PRINT(self->distance);
   //DEBUG_PRINT(" after microseconds: ");
   //DEBUG_PRINTLN(microDiff);
   distanceReady = true;
}

boolean lidarOnce(){
  int timeout = 0;
  //delayMicroseconds(100);
    Controller.spinOnce();
      while((!distanceReady) && timeout < 20){
        delayMicroseconds(100);
        timeout++;
      }
      if(timeout == 10){
        return(true); //error
      }
  return(false); // no error
}

void simpleScan(char* skipVar) {
  DEBUG_PRINTLN("Starting Simple Scan.");
  // Calculate how many microsteps to move between each reading
  // given number of steps in a full 360 turn of the device
  //int skipPitch;
  //int skipYaw;
  float skipDeg;
  int currDir = 1;
  boolean lidarError = false;
//  int timeout = 0;
  //char byteBuf[sizeof(reading)];
  
  if(strcmp(skipVar,"0.9")==0){
    //skipPitch = pitch.steps*pitch.microstep/400;
    //skipYaw = yaw.steps*yaw.microstep/400;
    skipDeg = 0.9;
//    if(skipPitch == 0 || skipYaw == 0){
//      DEBUG_PRINTLN("Resolution finer than microstep mode.");
//    }
  } else if(strcmp(skipVar,"0.45")==0){
    //skipPitch = pitch.steps*pitch.microstep/800;
    //skipYaw = yaw.steps*yaw.microstep/800;
    skipDeg = 0.45;
//    if(skipPitch == 0 || skipYaw == 0){
//      DEBUG_PRINTLN("Resolution finer than microstep mode.");
//    }
  } else if(strcmp(skipVar,"0.225")==0){
    //skipPitch = pitch.steps*pitch.microstep/1600;
    //skipYaw = yaw.steps*yaw.microstep/1600;
    skipDeg = 0.225;
//    if(skipPitch == 0 || skipYaw == 0){
//      DEBUG_PRINTLN("Resolution finer than microstep mode.");
//    }
  } else {
    DEBUG_PRINTLN("Can only scan in increments of 0.225,0.45,or 0.9.");
    return;
  }

  enableMotors();

  // Find a filename to use
  findFileName();

  // Open the file for writing
  if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
    DEBUG_PRINTLN("Error Opening File");
    error("file.open");
    return;
  }
  
  // Write a header
  writeHeader();

  

  // Assume that the device starts pointing straight down.
  // Assume that the device has a blindspot straight down of ~36 deg.
  // So we need to move pitch start point over by 0.225*100 = 22.5 deg.
  // This is close to a blindspot of 45 deg and is extra conservative
  // This is the same as saying the device needs to move over 1/16 of 
  // the number of steps in a rotation.
  //int startOffset = pitch.steps*pitch.microstep/16;
  float startOffset = 22.5; //degrees
  //int maxPitchSteps = (pitch.steps*pitch.microstep) - (2*startOffset);
  float maxPitchDeg = 360.0 - startOffset;

  // Yaw only has to turn half-way around because pitch goes up and over
  //int maxYawSteps = (yaw.steps*yaw.microstep)/2;
  float maxYawDeg = 180.0;

  lastMicro = micros();

  // Move to starting point
  //pitchMot.move(startOffset);
  pitchMot.rotate(startOffset);

  DEBUG_PRINT("Motor offset took: ");
  DEBUG_PRINTLN(micros()-lastMicro);

  lastMicro = micros();

  int countError = 0;
  yield();
  // Clear lidar buffer and warm up sensor
  for(int k = 0; k < 1000; k++){
    //timeout = 0;
    distanceReady = false;
    //delayMicroseconds(100);
//    Controller.spinOnce();
//      while((!distanceReady) && timeout < 20){
//        delayMicroseconds(100);
//        timeout++;
//      }
//      if(timeout == 10){
//        countError++;
//      }
    lidarError = lidarOnce();
    if(lidarError){
      countError++;
    }
  }

  DEBUG_PRINT("Warmup and Clearing Controller took: ");
  DEBUG_PRINTLN(micros()-lastMicro);
  DEBUG_PRINT("Error readings count: ");
  DEBUG_PRINTLN(countError);

  lastMicro = micros();
  
  // scan first point, then enter loop
  //Controller.spinOnce();
  //delayMicroseconds(shotMicroDelay);
  lidarError = lidarOnce();
    if(!lidarError){
      reading.radius = LZ1.distance;
      reading.intensity = LZ1.strength;
      reading.pitch = 360.0*(float)(startOffset)/float(pitch.steps*pitch.microstep);
      reading.yaw = 0.0;
    }
  
  DEBUG_PRINT("First Lidar Shot took: ");
  DEBUG_PRINTLN(micros()-lastMicro);

  lastMicro = micros();
  
  if(!lidarError){
    file.print(reading.radius);
    file.print(",");
    file.print(reading.yaw);
    file.print(",");
    file.print(reading.pitch);
    file.print(",");
    file.print(reading.intensity);
    file.print("\n");
  }
  else {
    DEBUG_PRINTLN("Unable to write to SD because of lidar error.");
  }
  
  //write to sd card
  // Convert to bytes
  //memcpy(byteBuf, &reading, sizeof(reading));
  // write
  //file.write(byteBuf,sizeof(byteBuf));

  DEBUG_PRINT("First SD write took: ");
  DEBUG_PRINTLN(micros()-lastMicro);

  lastMicro = micros();
  
  // for each yaw
  //for testing purposes limit maxYawSteps artificially
  maxYawDeg = 1.8;
  for(float i=0; i<=maxYawDeg; i=i+skipDeg){
    // Check for cancel command
    checkSerial();
    if(cancel){ 
      file.close();
      return; 
    }
    
    // Clear lidar buffer
    Controller.spinOnce();
    delayMicroseconds(shotMicroDelay);
    
    DEBUG_PRINT("Scanning yaw ");
    //DEBUG_PRINT(360.0*(float)(i)/float(yaw.steps*pitch.microstep));
    DEBUG_PRINT(i);
    DEBUG_PRINT(" at microseconds ");
    DEBUG_PRINT(micros() - lastMicro);
    DEBUG_PRINT(": ");
    //laserprint();
    if(!lidarError){
      if(!lidarError){
        DEBUG_PRINT(reading.radius);
        DEBUG_PRINT(",");
        DEBUG_PRINT(reading.yaw);
        DEBUG_PRINT(",");
        DEBUG_PRINT(reading.pitch);
        DEBUG_PRINT(",");
        DEBUG_PRINT(reading.intensity);
        DEBUG_PRINT("\n");
      }
    }
    
    // perform a full pitch scan
    
    for(float j=startOffset; j<maxPitchDeg; j=j+skipDeg){
      //move pitch motor
      //pitchMot.move(currDir*skipPitch);
      pitchMot.rotate(currDir*skipDeg);
      
      //take a reading
      lidarError = lidarOnce();
      if(!lidarError){
        reading.radius = LZ1.distance;
        reading.intensity = LZ1.strength;
        reading.pitch = j;
        reading.yaw = i;
        file.print(reading.radius);
        file.print(",");
        file.print(reading.yaw);
        file.print(",");
        file.print(reading.pitch);
        file.print(",");
        file.print(reading.intensity);
        file.print("\n");
      }
      //Controller.spinOnce();
      //delayMicroseconds(shotMicroDelay);
      //reading.radius = LZ1.distance;
      //reading.intensity = LZ1.strength;
      //reading.pitch = j;
      //reading.yaw = i;

      //file.print(reading.radius);
      //file.print(",");
      //file.print(reading.yaw);
      //file.print(",");
      //file.print(reading.pitch);
      //file.print(",");
      //file.print(reading.intensity);
      //file.print("\n");
    }
    // after pitch done change direction of pitch
    currDir = -1*currDir;
    //move yaw motor
    yawMot.move(skipDeg);
  }

  // close SD file
  file.close();

  currMicro = micros();

  disableMotors();

  DEBUG_PRINTLN("Simple Scan Complete.");
  DEBUG_PRINT("Microseconds to complete: ");
  DEBUG_PRINT(currMicro-lastMicro);
  DEBUG_PRINTLN();
}
