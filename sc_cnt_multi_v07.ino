/*
 Scanner Control
 Date: 04/20/2014



 This toggles the motors using the motors.
 Based on Arduino Mega 2560

 Controls 12 motor
 The user enters the index of the number starting from 1.
 Or the user can also enter two numbers which would be the index of row and column both starting from 1

  Pin orderfor a given motor is
  Left, right, Up, Down

  Once entering the state: The user can control the motors using

  'a': Left
  's': Right
  'w': Up
  'z': Down

  Other letters stops all the motors
   To enter index: ENter like "3 4\n" for two index (row, col0)=(3,4) or just "2\n" to control the second motor
 */

#define DEBUGFLAG 0



//Input String from Serial
String inputString = "";
boolean stringComplete = false;

//End of Input Sring Indicator
const char endStringChar = '\n';


//selected motor Row and Column index
int selectedRow = 0;
int selectedCol = 0;


//temporary numbers to parse string
int parse_no1;
int parse_no2;
int tempVar;


//# of motors in row and column
const int nRowMotor = 4;
const int nColMotor = 3;

// States of motors initially
int LeftMotorState[nRowMotor][nColMotor] = {LOW};
int RightMotorState[nRowMotor][nColMotor] = {LOW};
int UpMotorState[nRowMotor][nColMotor] = {LOW};
int DownMotorState[nRowMotor][nColMotor] = {LOW};


// pin Assignment for motor 1 at (row, col) = (1,1)
const int AzimLeftPin_init = 2;
const int AzimRightPin_init = 3;
const int ElevUpPin_init = 4;
const int ElevDownPin_init = 5;



short MotorPinList[nRowMotor][nColMotor][4] = {
  AzimLeftPin_init, AzimRightPin_init, ElevUpPin_init, ElevDownPin_init
};



void setup() {
  // initialize serial communication:
  Serial.begin(9600);

  //Reserve 10 bytes for the input String
  inputString.reserve(10);

  delay(1000);
  //initialize the motor pin values

  // initialize the motor pins

  for (int RowIndex = 0; RowIndex < nRowMotor ; RowIndex++)
    for (int ColIndex = 0; ColIndex < nColMotor ; ColIndex++)
    {
      for (int motorIndex = 0; motorIndex < 4 ; motorIndex++)
      {
        int PinOffsetTemp =  ((RowIndex * nColMotor) + ColIndex) * 4 + motorIndex;
        MotorPinList[RowIndex][ColIndex][motorIndex] = AzimLeftPin_init + PinOffsetTemp;
      }
#if DEBUGFLAG
      Serial.print("Row:");
      Serial.print(RowIndex + 1);
      Serial.print("  Col:");
      Serial.print(ColIndex + 1);
      Serial.print("\t---: ");

      for (int motorIndex = 0; motorIndex < 4 ; motorIndex++)
      {
        Serial.print(MotorPinList[RowIndex][ColIndex][motorIndex]);
        Serial.print('\t');
      }
      Serial.println();
#endif


    }
  // initialize the motor pins as OUTPUT:


  for (int RowIndex = 0; RowIndex < nRowMotor ; RowIndex++)
    for (int ColIndex = 0; ColIndex < nColMotor ; ColIndex++)
      for (int motorIndex = 0; motorIndex < 4 ; motorIndex++)
      {
        pinMode(MotorPinList[RowIndex][ColIndex][motorIndex], OUTPUT);
      }


  // Turn off all the motors
  TurnOffAllMotors();

  //Print SelectMotor
  printSelectMirror();

}



void loop() {


  //Serial.println("Ready for Serial:");



  // read the sensor:
  if (Serial.available() > 0) {
    int inByte = Serial.read();






    switch (inByte) {
      case 'a':
        //toggle left motor
        LeftMotorState[selectedRow][selectedCol] = !LeftMotorState[selectedRow][selectedCol];

        //make sure you do not excite the Right at the same time
        // if the left motor is on.
        // Running this first to avoid excitement of two motors at the same time.
        update_motor_pair(LeftMotorState[selectedRow][selectedCol],
                          &RightMotorState [selectedRow][selectedCol],
                          MotorPinList[selectedRow][selectedCol][1]);
        digitalWrite(MotorPinList[selectedRow][selectedCol][0], LeftMotorState[selectedRow][selectedCol]);
        print_state(selectedRow, selectedCol);
        break;

      case 's':
        //toggle right motor
        RightMotorState[selectedRow][selectedCol] = !RightMotorState[selectedRow][selectedCol];

        //make sure you do not excite the left at the same time
        // if the right motor is on.
        // Running this first to avoid excitement of two motors at the same time.
        update_motor_pair(RightMotorState[selectedRow][selectedCol],
                          &LeftMotorState [selectedRow][selectedCol],
                          MotorPinList[selectedRow][selectedCol][0]);
        digitalWrite(MotorPinList[selectedRow][selectedCol][1], RightMotorState[selectedRow][selectedCol]);
        print_state(selectedRow, selectedCol);
        break;

      case 'w':
        //toggle up motor
        UpMotorState[selectedRow][selectedCol] = !UpMotorState[selectedRow][selectedCol];

        //make sure you do not excite the Down at the same time
        // if the Up motor is on.
        // Running this first to avoid excitement of two motors at the same time.
        update_motor_pair(UpMotorState[selectedRow][selectedCol],
                          &DownMotorState [selectedRow][selectedCol],
                          MotorPinList[selectedRow][selectedCol][3]);
        digitalWrite(MotorPinList[selectedRow][selectedCol][2], UpMotorState[selectedRow][selectedCol]);

        print_state(selectedRow, selectedCol);
        break;

      case 'z':
        //toggle down motor
        DownMotorState[selectedRow][selectedCol] = !DownMotorState[selectedRow][selectedCol];

        //make sure you do not excite the Up at the same time
        // if the Down motor is on.
        // Running this first to avoid excitement of two motors at the same time.
        update_motor_pair(DownMotorState[selectedRow][selectedCol],
                          &UpMotorState [selectedRow][selectedCol],
                          MotorPinList[selectedRow][selectedCol][2]);
        digitalWrite(MotorPinList[selectedRow][selectedCol][3], DownMotorState[selectedRow][selectedCol]);
        print_state(selectedRow, selectedCol);
        break;

      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
      case '0':
      case ' ':
        if (inputString.length() < 10)
          inputString = inputString + (char) inByte;
        else
          stringComplete = true;

        break;
      case endStringChar:
        // only end the string if you gathered some numbers
        if (inputString.length() > 0)
          stringComplete = true;


        break;

      default:
        // turn all the Motors off:
        TurnOffAllMotors();
        Serial.println("All Off!");
        print_state(selectedRow, selectedCol);

    } //switch




    if (stringComplete) {

      //find the whitespace index
      tempVar = inputString.indexOf(' ');



      // If there is a whitespace
      if (tempVar >= 0) {
        parse_no1 = inputString.substring(0, tempVar).toInt();
        parse_no2 = inputString.substring(tempVar + 1).toInt();

      }
      else
      {
        parse_no1 = inputString.substring(0, tempVar).toInt();

        //parse_no2=-1 indicates that we will just use the number one as the single number index
        //parse_n1=
        parse_no2 = -1;

      }

      /*
       Serial.print ("Integer 1 is ");
       Serial.println(parse_no1);
       Serial.print ("Integer 2 is ");
       Serial.println(parse_no2);
      Serial.println("Checking Range:");

      */
      updateSelectMirror(parse_no1, parse_no2);



      // reset the String
      stringComplete = false;
      inputString = "";
      parse_no1 = 0;
      parse_no2 = 0;

    } // if stringComplete

  } // if serial




} //while loop


//update_motor_pair() is called on two states with oppoins direction (left,right) and (up,down)
//Ensures that if first motor is high the second motor is off, and also updates the states
void update_motor_pair(const int FirstMotorState, int * const SecondMotorState_pt, const int SecondMotorPin)

{
  if (FirstMotorState == HIGH)
    *SecondMotorState_pt = !FirstMotorState;
  digitalWrite(SecondMotorPin, *SecondMotorState_pt);
}



// Prints the states of all the motors
void print_state(const int RowIndex, const int ColIndex) {
  Serial.println("Left\tRight\tUp\tDown");
  Serial.print(LeftMotorState[RowIndex][ColIndex]);
  Serial.print("\t");
  Serial.print(RightMotorState[RowIndex][ColIndex]);
  Serial.print("\t");
  Serial.print(UpMotorState[RowIndex][ColIndex]);
  Serial.print("\t");
  Serial.println(DownMotorState[RowIndex][ColIndex]);

}

void TurnOffAllMotors()
{
  for (int RowIndex = 0; RowIndex < nRowMotor ; RowIndex++)
    for (int ColIndex = 0; ColIndex < nColMotor ; ColIndex++)
    {
      LeftMotorState[RowIndex][ColIndex] = LOW;
      RightMotorState[RowIndex][ColIndex] = LOW;
      UpMotorState[RowIndex][ColIndex] = LOW;
      DownMotorState[RowIndex][ColIndex] = LOW;

      for (int motorIndex = 0; motorIndex < 4 ; motorIndex++)
      {
        digitalWrite(MotorPinList[RowIndex][ColIndex][motorIndex], LOW);
      }
    }

}


// Following function checks for the range of parseTemp1 and parseTemp2 which are the two consecutive
//parsed integers, and updates the current mirror selection
bool updateSelectMirror(const int parseTemp1,  const int parseTemp2)
{
  bool inRangeFlag = false;
  if (parseTemp2 == -1) //then first number will be treated as a whole index
  {
    //single array index must be less than the number of all the mirrors
    inRangeFlag = (parseTemp1 > 0) && (parseTemp1 <= nRowMotor * nColMotor);
    if (inRangeFlag)
    {
      selectedCol = (parseTemp1 - 1) % nColMotor;
      selectedRow = (parseTemp1 - 1) / nColMotor;

    }
  }

  else  // separately check the numbers if they are within row and col range

  {
    inRangeFlag = (parseTemp1 <= nRowMotor) && (parseTemp1 > 0) &&
                  (parseTemp2 <= nColMotor) && (parseTemp2 > 0)  ;
    if (inRangeFlag)
    {
      selectedCol = parseTemp2 - 1;
      selectedRow = parseTemp1 - 1;
    }
  }

  if (inRangeFlag == true)
  {
    printSelectMirror();
  }
  else
  {
    Serial.println("Index out of range!");
    printSelectMirror();
  }
  return inRangeFlag;
}

// Print Selected Mirror
void printSelectMirror(void) {
  Serial.print("Selected Row:");
  Serial.print(selectedRow + 1);
  Serial.print(" Column: ");
  Serial.println(selectedCol + 1);
  print_state(selectedRow, selectedCol);
}
