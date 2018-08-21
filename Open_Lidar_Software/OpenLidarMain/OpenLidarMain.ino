
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
#include <DRV8825.h>

// Define whether or not to use DEBUG mode

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINTLN(x)  Serial.println (x)
 #define DEBUG_PRINT(x) Serial.print (x)
#else
 #define DEBUG_PRINT(x) // do nothing
 #define DEBUG_PRINTLN(x) // do nothing
#endif

#define TerminalBAUD 9600  //38400    //Baud rate of serial or bluetooth output

// Define serial command object
#define mySerial Serial // Define to serial you want to use...Serial for usb, Serial5 for bluetooth
const byte numChars = 32;
char receivedChars[numChars];
boolean receiveFull = false;
#define MAX_WORD_COUNT 3
#define MIN_WORD_COUNT 2
char *Words[MAX_WORD_COUNT];
boolean lockSerial = false; // Some functions may want to allow only "Cancel" as a command
boolean cancel = false; // Functions that allow cancel will check for this flag each major loop

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

motorStruct pitch = {2,3,4,5,6,7,8,9,400,4,30};
motorStruct yaw = {12,24,25,26,27,28,29,30,8000,4,1};

// Initiate the motors using the DRV8825 library
DRV8825 pitchMot(pitch.steps,pitch.DIR,pitch.STP,pitch.EN,pitch.M0,pitch.M1,pitch.M2);
DRV8825 yawMot(yaw.steps,yaw.DIR,yaw.STP,yaw.EN,yaw.M0,yaw.M1,yaw.M2);


/*
* Setup Function - Run once
*/
void setup() {
  // put your setup code here, to run once:
  // Set serial communication over usb
  mySerial.begin(TerminalBAUD);
  // Set serial for bluetooth
  //Serial5.begin(TerminalBAUD);
  //while (!Serial);  //Wait for serial before doing anything else
  
  // define pins numbers
  // Motor 1A (pitch) connected to digital output pins
  // EN;M0;M1;M2;RST;SLP;STP;DIR;steps;microstep;rpm; 
//  pitch = {2,3,4,5,6,7,8,9};
//  pitch.steps = 400;
//  pitch.microstep = 4;
//  pitch.rpm = 30;
  
  // Motor 0.6A (yaw) connected to digital output pins
  // EN;M0;M1;M2;RST;SLP;STP;DIR;steps;microstep;rpm; 
//  yaw = {12,24,25,26,27,28,29,30};
//  yaw.steps = 200 * 40; // The yaw motor is connected to a 40:1 worm gear
//  yaw.microstep = 4;
//  yaw.rpm = 1; // 30/40 as integer

  //mot.begin(rpm,microsteps)
  //pitchMot.begin(pitchRPM, pitchStepMode);
  //yawMot.begin(yawRPM, yawStepMode);
  //pitchMot.begin(pitch.rpm,pitch.microstep);
  //yawMot.begin(yaw.rpm,yaw.microstep);

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

  DEBUG_PRINTLN(F("Setup complete"));
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

void checkSerial(){
  static byte ndx = 0;
  char endMarker = '\n';
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
    receiveFull = false;
  }
}

void parseCommand(){
  DEBUG_PRINT(F("Parsed Command:"));
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
    if(strcmp(Words[0],"pitch") == 0){
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
    }
    else if(strcmp(Words[0],"bluetooth") == 0){
      DEBUG_PRINTLN("Bluetooth received.");
    }
    else if(strcmp(Words[0],"sd") == 0){
      DEBUG_PRINTLN("SD received.");
    }
    else if(strcmp(Words[0],"motor") == 0){
      DEBUG_PRINTLN("Motor received.");
      if(strcmp(Words[1],"enable")==0) enableMotors();
      else if(strcmp(Words[1],"disable")==0) disableMotors();
    }
  }
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

/*
 * Parse a C string
 */
//inline void split(char* inVal, char outVal[NUM_WORDS][STRING_LEN])
//{
//    int i = 0;
//    char *p = strtok(inVal, " ,/");
//    strcpy(&outVal[i++][0], p);
//    while (p)
//    {
//      p = strtok(NULL, " ,/");
//      if (p)
//      {
//          strcpy(&outVal[i++][0], p);
//      }
//    }
//}

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
