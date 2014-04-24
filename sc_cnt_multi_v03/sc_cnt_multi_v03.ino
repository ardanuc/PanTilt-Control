/*
 Scanner Control
 Date: 04/20/2014
 
 This toggles the motors using the motors.
 Based on Arduino Mega 2560
 
 Controls 12 motor
 
  Pin orderfor a given motor is 
  Left, right, Up, Down
 */

#define DEBUGFLAG 0

 
// Both azimuth and elevation motors can be in two states
// For Azimuth (OFF, LEFT, RIGHT) corresponds to (OFF, TURNA, TURNB)
// For Elevation (OFF, UP, DOWN)  corresponsd to (OFF, TURNA, TURNB)
enum MotorState {OFF, TURNA, TURNB};

//

//# of motors in row and column
const int nRowMotor = 3;
const int nColMotor = 3;


MotorState AzimStateArr [nRowMotor][nColMotor] = {OFF};
MotorState ElevStateArr [nRowMotor][nColMotor] = {OFF};

// pin Assignment for motor 1 at (row, col) = (1,1) 
const int AzimPinA_init=2;
const int AzimPinB_init=3;
const int ElevPinA_init=4;
const int ElevPinB_init=5;


//AzimPinA 
int AzimPin_A_List[nRowMotor][nColMotor]={AzimPinA_init};
int AzimPin_B_List[nRowMotor][nColMotor]={AzimPinB_init};
int ElevPin_A_List[nRowMotor][nColMotor]={ElevPinA_init};
int ElevPin_B_List[nRowMotor][nColMotor]={ElevPinB_init};
  



void setup() {
  // initialize serial communication:
  Serial.begin(9600); 
  delay(1000);
  //initialize the motor pin values
  
    // initialize the motor pins 
  for (int RowIndex= 0; RowIndex <nRowMotor ; RowIndex++) 
  for (int ColIndex= 0; ColIndex <nColMotor ; ColIndex++)
  {
    int PinOffsetTemp=  ((RowIndex*nRowMotor)+ColIndex)*4;
    AzimPin_A_List[RowIndex][ColIndex]=AzimPinA_init+PinOffsetTemp;
    AzimPin_B_List[RowIndex][ColIndex]=AzimPinB_init+PinOffsetTemp;
    ElevPin_A_List[RowIndex][ColIndex]=ElevPinA_init+PinOffsetTemp;
    ElevPin_B_List[RowIndex][ColIndex]=ElevPinB_init+PinOffsetTemp;

  
   #if DEBUGFLAG
    Serial.print("Row:");
    Serial.print(RowIndex+1);
    Serial.print("  Col:");
    Serial.print(ColIndex+1);
    Serial.print("\t---: ");
    
    Serial.print(AzimPin_A_List[RowIndex][ColIndex]);
    Serial.print('\t');
        Serial.print(AzimPin_B_List[RowIndex][ColIndex]);
    Serial.print('\t');
        Serial.print(ElevPin_A_List[RowIndex][ColIndex]);
    Serial.print('\t');
        Serial.print(ElevPin_B_List[RowIndex][ColIndex]);
    Serial.println('\t');
   #endif 
   
  }   
  
  // initialize the motor pins as OUTPUT:
  for (int RowIndex= 0; RowIndex <nRowMotor ; RowIndex++) 
  for (int ColIndex= 0; ColIndex <nColMotor ; ColIndex++)
  {
    pinMode(AzimPin_A_List[RowIndex][ColIndex], OUTPUT);
     pinMode(AzimPin_B_List[RowIndex][ColIndex], OUTPUT);
      pinMode(ElevPin_A_List[RowIndex][ColIndex], OUTPUT);
       pinMode(ElevPin_B_List[RowIndex][ColIndex], OUTPUT);
  }     
  
  

 
}


void loop() {
  
  delay(1000);
  Serial.println("You are the man;");
    
    
    ElevStateArr[0][0]= TURNB;
    RefreshMotorAzim_ij(0, 0);
    RefreshMotorElev_ij(0, 0);
  
  while (1);
  /*
  
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

*/
}





// Refreshes the Azim motor state by driving the motor at 
// (RowIndex,ColIndex) with appropriate State
void RefreshMotorAzim_ij(const int RowIndex, const int ColIndex)
{
  switch(AzimStateArr[RowIndex][ColIndex])
  {
  case TURNA:
   digitalWrite(AzimPin_A_List[RowIndex][ColIndex], HIGH);
   digitalWrite(AzimPin_B_List[RowIndex][ColIndex], LOW);
   break;
   
  case TURNB:
   digitalWrite(AzimPin_A_List[RowIndex][ColIndex], LOW);
   digitalWrite(AzimPin_B_List[RowIndex][ColIndex], HIGH); 
   break;
  
  case OFF:
  default:
   digitalWrite(AzimPin_A_List[RowIndex][ColIndex], LOW);
   digitalWrite(AzimPin_B_List[RowIndex][ColIndex], LOW); 
  }
}

// Refreshes the Elev motor state by driving the motor at 
// (RowIndex,ColIndex) with appropriate State
void RefreshMotorElev_ij(const int RowIndex, const int ColIndex)
{
  switch(ElevStateArr[RowIndex][ColIndex])
  {
  case TURNA:
   digitalWrite(ElevPin_A_List[RowIndex][ColIndex], HIGH);
   digitalWrite(ElevPin_B_List[RowIndex][ColIndex], LOW);
   break;
   
  case TURNB:
   digitalWrite(ElevPin_A_List[RowIndex][ColIndex], LOW);
   digitalWrite(ElevPin_B_List[RowIndex][ColIndex], HIGH); 
   break;
  
  case OFF:
  default:
   digitalWrite(ElevPin_A_List[RowIndex][ColIndex], LOW);
   digitalWrite(ElevPin_B_List[RowIndex][ColIndex], LOW); 
  }
}

/*

//update_motor_pair() is called on two states with oppoins direction (left,right) and (up,down)
//Ensures that if first motor is high the second motor is off, and also updates the states
void update_motor_pair(const int FirstMotorState, int * const SecondMotorState_pt, const int SecondMotorPin)

{
  if (FirstMotorState==HIGH)
    *SecondMotorState_pt=!FirstMotorState;
  digitalWrite(SecondMotorPin, *SecondMotorState_pt);  
}


*/
