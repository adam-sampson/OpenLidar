/*     Simple Stepper Motor Control Exaple Code
 *      
 *  
 */
#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
 #define BAUD (115200)
#else
 #define DEBUG_PRINT(x)
#endif
 
// defines pins numbers
const int stepPin = 5; 
const int dirPin = 4; 
const int enablePin = 6;

const int riseDelay = 500;
const int fallDelay = 500;
 
void setup() {
  #ifdef DEBUG
    Serial.begin(BAUD);
  #endif
  
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enablePin,OUTPUT);

  digitalWrite(enablePin,LOW);
}
void loop() {
  #ifdef DEBUG
    unsigned long microcounter;
    microcounter = micros();
  #endif
  
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(riseDelay); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(fallDelay); 
  }

  #ifdef DEBUG
    microcounter = micros() - microcounter;
    DEBUG_PRINT(microcounter);
  #endif
  
  delay(1000); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making one full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(riseDelay);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(fallDelay);
  }
  delay(1000);
}
