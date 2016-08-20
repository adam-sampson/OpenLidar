/*     Open Lidar Project Code
 *     Author: Adam Sampson
 *     
 */

// #define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
 #define BAUD (115200)
#else
 #define DEBUG_PRINT(x)
#endif

// define pins numbers
const int stepPin1 = 3; 
const int dirPin1 = 4; 
const int stepPin2 = 5; 
const int dirPin2 = 6; 

// define motor profile
const int riseDelay = 250;
const int fallDelay = 250;
int myYawSteps = 400;  // This is a 400 step motor
int myPitchSteps = 400 * 4;  // This is a 400 step motor on quarter step mode

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
    Serial.begin(BAUD);
  #endif

  // Sets the two pins as Outputs
  pinMode(dirPin1,OUTPUT);
  pinMode(stepPin1,OUTPUT); 
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin2,OUTPUT); 
}

void loop() {
    unsigned long microcounter;
    microcounter = micros();

  delay(10000); // 10 second delay to get hands out of the way

// For each full rotation of the pitch motor run the yaw motor one step
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
}

void StepMotorForward(int myStepPin, int myDirPin){
  digitalWrite(myDirPin,HIGH);
  digitalWrite(myStepPin,HIGH);
  delayMicroseconds(riseDelay);
  digitalWrite(myStepPin,LOW);
  delayMicroseconds(fallDelay);
}

