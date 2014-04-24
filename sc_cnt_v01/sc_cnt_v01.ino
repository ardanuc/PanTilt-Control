/*
 Scanner Control
 Date: 04/16/2014
 
 This will read a key from the keyboard and echo it
 */

/*
  Switch statement  with serial input
 
 Demonstrates the use of a switch statement.  The switch
 statement allows you to choose from among a set of discrete values
 of a variable.  It's like a series of if statements.
 
 To see this sketch in action, open the Serial monitor and send any character.
 The characters a, b, c, d, and e, will turn on LEDs.  Any other character will turn
 the LEDs off.
 
 The circuit:
 * 5 LEDs attached to digital pins 2 through 6 through 220-ohm resistors
 
 created 1 Jul 2009
 by Tom Igoe 
 
 */

// pin Assignment
const int LeftMotorPin=2;
const int RightMotorPin=3;
const int UpMotorPin=4;
const int DownMotorPin=5;


const int MotorPinList[]={
  LeftMotorPin,RightMotorPin,UpMotorPin,DownMotorPin};

// States of motors initially
int LeftMotorState=LOW;
int RightMotorState=LOW;
int UpMotorState=LOW; 
int DownMotorState=LOW;



void setup() {
  // initialize serial communication:
  Serial.begin(9600); 
  // initialize the LED pins:
  for (int motorIndex = 0; motorIndex <4 ; motorIndex++) {
    pinMode(MotorPinList[motorIndex], OUTPUT);
  }     
  // turn all the Motors off intially:
  for (int motorIndex = 0; motorIndex <4 ; motorIndex++) {
    digitalWrite(MotorPinList[motorIndex], LOW);
  } 
}

void loop() {
  // read the sensor:
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    // do something different depending on the character received.  
    // The switch statement expects single number values for each case;
    // in this exmaple, though, you're using single quotes to tell
    // the controller to get the ASCII value for the character.  For 
    // example 'a' = 97, 'b' = 98, and so forth:

    switch (inByte) {
    case 'a':    
      //toggle left motor
      LeftMotorState=!LeftMotorState;
      digitalWrite(LeftMotorPin, LeftMotorState);
      //Serial.println(inByte);
      //Serial.println(LeftMotorState);
      break;
    case 's':    
      //toggle right motor
      RightMotorState=!RightMotorState;
      digitalWrite(RightMotorPin, RightMotorState);
      break;  
    case 'w':    
      //toggle up motor
      UpMotorState=!UpMotorState;
      digitalWrite(UpMotorPin, UpMotorState);
      break; 
    case 'z':    
      //toggle down motor
      DownMotorState=!DownMotorState;
      digitalWrite(DownMotorPin, DownMotorState);
      break;     


    default:
      // turn all the Motors off:
      for (int motorIndex = 0; motorIndex <4 ; motorIndex++) {
        digitalWrite(MotorPinList[motorIndex], LOW);
      }
    } 
  }
}


