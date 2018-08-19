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
 */

// Define Libraries
//#include <BasicStepperDriver.h>
#include <DRV8825.h>
//#include <MultiDriver.h>
//#include <SyncDriver.h>

// Define whether or not to use DEBUG mode

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINTLN(x)  Serial.println (x)
 #define DEBUG_PRINT(x) Serial.print (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

#define TerminalBAUD 9600  //38400    //Baud rate of serial or bluetooth output

// define an array of characters to read serial into for bluetooth commands
String serialData;

// define pins numbers
// Motor 1A (pitch) connected to digital output pins
#define pitchEN  2
#define pitchM0  3
#define pitchM1  4
#define pitchM2  5
#define pitchRST 6
#define pitchSLP 7
#define pitchSTP 8
#define pitchDIR 9

// Motor 0.6A (yaw) connected to digital output pins
#define yawEN  12
#define yawM0  24
#define yawM1  25
#define yawM2  26
#define yawRST 27
#define yawSLP 28
#define yawSTP 29
#define yawDIR 30

// define motor profile
//const int PROGMEM riseDelay1 = 2;
//const int PROGMEM fallDelay1 = 2;
//const int PROGMEM riseDelay2 = 2; 
//const int PROGMEM fallDelay2 = 2;
//const unsigned long PROGMEM pitchMotDelay = 300;
//const unsigned long PROGMEM yawMotDelay = 300; 

#define pitchStpPerDeg 400   // This is a 400 step motor aka 0.9deg
#define pitchStepMode 4      //Quarter step mode
#define pitchRPM 60
#define yawStpPerDeg 200*40  // This is a 200 step motor aka 1.8deg and hooked up to a 40:1 worm gear
#define yawStepMode 2        //Quarter step mode
#define yawRPM 60

// Initiate the motors using the DRV8825 library
DRV8825 pitchMot(pitchStpPerDeg,pitchDIR,pitchSTP,pitchEN,pitchM0,pitchM1,pitchM2);
DRV8825 yawMot(yawStpPerDeg,yawDIR,yawSTP,yawEN,yawM0,yawM1,yawM2);

/*
* Setup Function - Run once
*/
void setup() {
  // put your setup code here, to run once:
  // Set serial communication over usb
  Serial.begin(TerminalBAUD);
  // Set serial for bluetooth
  //Serial5.begin(TerminalBAUD);
  //while (!Serial);  //Wait for serial before doing anything else

  //mot.begin(rpm,microsteps)
  //pitchMot.begin(pitchRPM, pitchStepMode);
  //yawMot.begin(yawRPM, yawStepMode);
  pitchMot.begin(30,1);
  yawMot.begin(1,1);
  pitchMot.disable();
  yawMot.disable();

  pinMode(pitchSLP, OUTPUT);
  pinMode(pitchRST, OUTPUT);
  pinMode(yawSLP,   OUTPUT);
  pinMode(yawRST,   OUTPUT);
  digitalWrite(pitchSLP,HIGH);
  digitalWrite(yawSLP,  HIGH);
  digitalWrite(pitchRST,HIGH);
  digitalWrite(yawRST  ,HIGH);  
  
  //pitchMot = new DRV8825(pitchStpPerDeg,pitchDIR,pitchSTP,pitchEN,pitchM0,pitchM1,pitchM2);
  //yawMot = new DRV8825(yawStpPerDeg,yawDIR,yawSTP,yawEN,yawM0,yawM1,yawM2);

//  // Initialize Motor Pins as outputs
//  pinMode(pitchEN ,OUTPUT);
//  pinMode(pitchM0 ,OUTPUT);
//  pinMode(pitchM1 ,OUTPUT);
//  pinMode(pitchM2 ,OUTPUT);
//  pinMode(pitchRST,OUTPUT);
//  pinMode(pitchSLP,OUTPUT);
//  pinMode(pitchSTP,OUTPUT);
//  pinMode(pitchDIR,OUTPUT);
//                
//  pinMode(yawEN   ,OUTPUT);
//  pinMode(yawM0   ,OUTPUT);
//  pinMode(yawM1   ,OUTPUT);
//  pinMode(yawM2   ,OUTPUT);
//  pinMode(yawRST  ,OUTPUT);
//  pinMode(yawSLP  ,OUTPUT);
//  pinMode(yawSTP  ,OUTPUT);
//  pinMode(yawDIR  ,OUTPUT);
//  
//  // Set motors to sleep mode to conserve power
//  digitalWrite(pitchSLP,LOW);
//  digitalWrite(yawSLP,  LOW);
//
//  // Set engable to low (enabled) for later
//  digitalWrite(pitchEN ,LOW);            
//  digitalWrite(yawEN   ,LOW);
//  
//  // Set mode pins to LOW for full step default
//  digitalWrite(pitchM0 ,LOW);
//  digitalWrite(pitchM1 ,LOW);
//  digitalWrite(pitchM2 ,LOW); 
//  digitalWrite(yawM0   ,LOW);
//  digitalWrite(yawM1   ,LOW);
//  digitalWrite(yawM2   ,LOW);
//
//  // Set reset to high (enabled)
//  digitalWrite(pitchRST,HIGH);
//  digitalWrite(yawRST  ,HIGH);
//
//  // Set stp and dir to low. 
//  digitalWrite(pitchSTP,LOW); 
//  digitalWrite(yawSTP  ,LOW);
//  
//  digitalWrite(pitchDIR,LOW);
//  digitalWrite(yawDIR  ,LOW);

  DEBUG_PRINTLN(F("Setup complete"));
}

/*
* Loop Function - Run forever
*/
void loop() {
  // Set variables

  //delay(5000); // 5 second delay to get hands out of the way

  // If in debug mode isten for usb commands
  #ifdef DEBUG
  while(Serial.available()) {
    serialData = Serial.readString(); //read incoming data as a string
    DEBUG_PRINT("OL recieved: ");
    DEBUG_PRINTLN(serialData);
  }
  #endif
  
  //Listen for bluetooth commands
  while(Serial5.available()) {
    serialData = Serial5.readString(); //read incoming data as a string
    //DEBUG_PRINT("OL recieved: ");
    DEBUG_PRINTLN(serialData);
  }

  //Decide what to do with the command recieved
  if(serialData.startsWith(F("TestMotors"))) {
    pitchMot.enable();
    pitchMot.rotate(360);
    pitchMot.disable();

    yawMot.enable();
    yawMot.rotate(360);
    yawMot.disable();
    DEBUG_PRINTLN("Test Motor Function Run.");
    serialData = "";
  }
  else if(serialData.startsWith(F("MotorsOn"))) {
    pitchMot.enable();
    yawMot.enable();
    DEBUG_PRINTLN("Enable Motor Function Run.");
    serialData = "";
  }
  else if(serialData.startsWith(F("MotorsOff"))) {
    pitchMot.disable();
    yawMot.disable();
    DEBUG_PRINTLN("Disable Motor Function Run.");
    serialData = "";
  }
  else if (serialData.startsWith(F("ScanRoom"))) {
    delay(1);
    //scanRoom(); 
    //Unset the command string so we don't repeat next loop
    serialData = ""; 
  }
}


/*
* createNewFile function - as name says
* Check to see if file is taken and increment until unique name is found
* Return file name for use.
*/
//String createNewFileName() {
//  //This is a quick and dirty way to make sure a new file is created.
//  //In the future ROM will be used to track file numbers.
//  DEBUG_PRINTLN(F("Create FN"));
//  delay(100);
//  String baseFilename = "OL";
//  String tempFilename;
//  char tempBuffer[] = "OL1.CSV";
//
//  /*
//  if (mySdStatus = 0) {
//    DEBUG_PRINTLN(F("Initialization Failed."));
//  }
//  else {
//    DEBUG_PRINTLN(F("SD was setup"));
//  */
//  
//  for (int fileNumber = 1; fileNumber < 5; fileNumber++) {
//    //DEBUG_PRINTLN("ForLoop");
//    //delay(100);
//    tempFilename = baseFilename + String(fileNumber, DEC) + ".csv";
//    //tempFilename.toCharArray(tempBuffer,tempFilename.length()+1);
//    DEBUG_PRINT(F("Temp file: "));
//    DEBUG_PRINTLN(tempBuffer);
//    delay(100);
//    if (SD.exists(tempBuffer)) {  //if the file doesn't exist, return the name
//      DEBUG_PRINT(F("Found new filename: "));
//      delay(100);
//      DEBUG_PRINTLN(tempFilename);
//      return tempFilename;
//    }
//  //}
//  }
//  
//  //return "OL0001.csv";
//  DEBUG_PRINTLN(F("SD full")); 
//  delay(100);
//  return "";
//}
