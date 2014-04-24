/*
 Scanner Control
 Date: 04/16/2014
 
 This toggles the motors using the motors...llllllll
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

      //make sure you do not excite the Right at the same time
      // if the left motor is on.
      // Running this first to avoid excitement of two motors at the same time.
      update_motor_pair(LeftMotorState, &RightMotorState,RightMotorPin);
      digitalWrite(LeftMotorPin, LeftMotorState);

      print_state();
      break;
    case 's':    
      //toggle right motor
      RightMotorState=!RightMotorState;
      //make sure you do not excite the left motor at the same time
      // if the right motor is on
      // Running this first to avoid excitement of two motors at the same time.
      update_motor_pair(RightMotorState, &LeftMotorState,LeftMotorPin);
      digitalWrite(RightMotorPin, RightMotorState);

      print_state();
      break;  
    case 'w':    
      //toggle up motor
      UpMotorState=!UpMotorState;
      //make sure you do not excite the down motor at the same time
      // if the up motor is on
      // Running this first to avoid excitement of two motors at the same time.
      update_motor_pair(UpMotorState, &DownMotorState,DownMotorPin);
      digitalWrite(UpMotorPin, UpMotorState);

      print_state();
      break; 
    case 'z':    
      //toggle down motor
      DownMotorState=!DownMotorState;
      //make sure you do not excite the up motor at the same time
      // if the down motor is on
      // Running this first to avoid excitement of two motors at the same time.
      update_motor_pair(DownMotorState, &UpMotorState,UpMotorPin);
      digitalWrite(DownMotorPin, DownMotorState);


      print_state();
      break;     


    default:
      // turn all the Motors off:
      for (int motorIndex = 0; motorIndex <4 ; motorIndex++) {
        digitalWrite(MotorPinList[motorIndex], LOW);
      }
      print_state();

    } 
  }


}


// Prints the states of all the motors
void print_state() {
  Serial.println("Left\tRight\tUp\t\Down");
  Serial.print(LeftMotorState);
  Serial.print("\t");
  Serial.print(RightMotorState);
  Serial.print("\t");
  Serial.print(UpMotorState);
  Serial.print("\t");
  Serial.println(DownMotorState);

}

//update_motor_pair() is called on two states with oppoins direction (left,right) and (up,down)
//Ensures that if first motor is high the second motor is off, and also updates the states
void update_motor_pair(const int FirstMotorState, int * const SecondMotorState_pt, const int SecondMotorPin)

{
  if (FirstMotorState==HIGH)

    *SecondMotorState_pt=!FirstMotorState;
  digitalWrite(SecondMotorPin, *SecondMotorState_pt);  
}



