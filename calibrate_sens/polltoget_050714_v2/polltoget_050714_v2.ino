/*
 Scanner Control
 Date: 05/07/2014



ARduino is used to control and poll the data, It only responds the commands when they
follow the following format

CommandStartChar='$';
CommandEndChar='!';
So the format of the command is


<CommandStartChar><CommandString><CommandEndChar>

Options for the CommandString are
SR<space><number> Eg: If  'SR 1' It will select Row 1
SR?               Poll for the selected row, and Arduino will return the Sected Row


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


 NOTE: MAKE SURE THE series monitor i set to send newlie character or whatever the
 endStringChar is.l

 -----

 MEMSIC  tilt sensor:
 Memsic2125

   Read the Memsic 2125 two-axis accelerometer.  Converts the
   pulses output by the 2125 into milli-g's (1/1000 of earth's
   gravity) and prints them over the serial connection to the
   computer.

   The circuit:
	* X output of accelerometer to digital pin xpin (52)
	* Y output of accelerometer to digital pin ypin  (53)
	* +V of accelerometer to +5V
	* GND of accelerometer to ground

   http://www.arduino.cc/en/Tutorial/Memsic2125

   created 6 Nov 2008
   by David A. Mellis
   modified 30 Aug 2011
   by Tom Igoe

   This example code is in the public domain.


 ------
 FOR HMC5883 3-axis magnet sensor

 FILE:    ARD_HMC5803L_GY273_Example
   DATE:    23/10/13
   VERSION: 0.1

This is an example of how to use the Hobby Components GY-273 module (HCMODU0036) which
uses a Honeywell HMC5883L 3-Axis Digital Compass IC. The IC uses an I2C interface to
communicate which is compatible with the standard Arduino Wire library.

This example demonstrates how to initialise and read the module in single shot
measurement mode. It will continually trigger single measurements and output the
results for the 3 axis to the serial port.


CONNECTIONS:

MODULE    ARDUINO MEGA
VCC       3.3V
GND       GND
SCL       P21
SDA       P20
DRDY      N/A


You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used for the purpose of promoting or selling products
that directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
REASON WHATSOEVER.



 */

/* Include the standard Wire library */
#include <Wire.h>

/* The I2C address of the module */
#define HMC5803L_Address 0x1E

/* Register address for the X Y and Z data */
#define X 3
#define Y 7
#define Z 5

// Maximum length of the commands from the serial port including start and end characters
#define MAX_COMMAND_STRING_LENGTH 15
#define MAX_COMMAND_STRINGSTARTTEXT_LENGTH 5

#define DEBUGFLAG 0

#define  TILTEPSILON 6

// delay in milisecond the between consecutive  readings during lowpass
#define DELAY_LP_ACCELERATION 100 

//Input String from Serial
String inputString = "";
String commandString = "";
// commandStringStartText is the first part of the commandString until a space character or
// untill the end
String commandStringStartText = "";

boolean stringComplete = false;

//End of Input Sring Indicator
const char endStringChar = '\n';
//Putty terminal does not send newline, unlike serial monitor of ARduino
const char endStringChar_PuTTY = '\r';

//selected motor Row and Column index
int selectedRow = 0;
int selectedCol = 0;


//temporary numbers to parse strings or other tasks
int parse_no1 = 0;
int parse_no2 = 0;
int tempVar;

//
int inByte;   // Byte read from serial port

// Command start and end characters. They should be set to different characters
const char CommandStartChar = '$';
const char CommandEndChar = '!';

// index of the CommandStartChar and CommandEndChar in the inputString
short CommandStartIndex;
short CommandEndIndex;

//# of motors in row and column
const int nRowMotor = 1;
const int nColMotor = 2;

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



// these constants won't change:
const int xPin = 14;		// X output of the accelerometer
const int yPin = 15;		// Y output of the accelerometer

// define maximum tilt sensor readings
// normally they should come as part of a calibration procedure
const int MaxTiltReading = 752;
const int MinTiltReading = -856;


// Values for the tilt feedback control
// Tilt is alogned with x reading of the accelerometer, but in MEMSIC notation
// it is
int targetTiltReading;
int currentTiltReading;


// Low Pass Filter Characteristics
// See AN4248 from Freescale for implementation
// 1<<15 is 2^15
static unsigned int Nsamples_LP =8;    // number of samples used in moving average
static unsigned int alpha_LP=(1<<15)/Nsamples_LP; 
static long LowPassedData=0L;             // low passed variable

// variables to read the pulse widths:
int pulseX, pulseY;
// variables to contain the resulting accelerations
int accelerationX, accelerationY;

// variable to keep track of whhether the motor is active or not
bool motorActive = false;

short MotorPinList[nRowMotor][nColMotor][4] = {
  AzimLeftPin_init, AzimRightPin_init, ElevUpPin_init, ElevDownPin_init
};



void setup() {
  // initialize serial communication:
  Serial.begin(9600);

  //Reserve 10 bytes for the input String
  inputString.reserve(MAX_COMMAND_STRING_LENGTH);

  commandString.reserve(MAX_COMMAND_STRING_LENGTH - 2);

  //Allow maximum 5 letter command String Start Text
  commandStringStartText.reserve(MAX_COMMAND_STRINGSTARTTEXT_LENGTH);
  // initialize the pins connected to the accelerometer
  // as inputs:
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);



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

  /* Initialise the Wire library */
  Wire.begin();

  /* Initialise the module */
  Init_HMC5803L();

  //Print SelectMotor
  //printSelectMirror();

}



void loop() {





  // read the sensor:
  if (Serial.available() > 0) {
    inByte = Serial.read();

    // boolean of whether a valid character is entered or not, any number or letter and space characters are fine

    tempVar = ((inByte >= 'a') && (inByte <= 'z')) ||
              ((inByte >= '0') && (inByte <= '9')) ||
              ((inByte >= 'A') && (inByte <= 'Z')) ||
              (inByte == ' ') ||
              (inByte == CommandStartChar) ||
              (inByte == CommandEndChar) ||
              (inByte == '?');

    // When a valid character is entered
    if (tempVar)
    {
      // valid character
      if (inputString.length() < MAX_COMMAND_STRING_LENGTH)
        inputString = inputString + (char) inByte;
      else
        stringComplete = true;


    }
    else if ((inByte == endStringChar_PuTTY) || (inByte == endStringChar))
    {
      // only end the string if you gathered some numbers
      if (inputString.length() > 0)
        stringComplete = true;
    }
    else
    {
      // turn all the Motors off:
      TurnOffAllMotors();
      // Serial.println("All Off!");

    }


    if (stringComplete) {
      // Start Parsing
      /*This script assummes a command of the form
      <Text Command>[<space>< Number 1 >][<space><Number 2>]
      where <Number 1> and <Number 2> are optional. If they exist they are stored in
      variables parse_no1 and parse_no2, otherwise they are assigned a value of -
       */
      // Search for the start command and end command identifiers
      CommandStartIndex = inputString.indexOf(CommandStartChar);


      CommandEndIndex = inputString.indexOf(CommandEndChar);


#if DEBUGFLAG
      Serial.print ("Full String: ");
      Serial.println(inputString);
      Serial.print("Command Start at ");
      Serial.println(CommandStartIndex);
      Serial.print("Command End at ");
      Serial.println(CommandEndIndex);

#endif

      if (CommandEndIndex > CommandStartIndex + 1)
      {
        commandString = inputString.substring(CommandStartIndex + 1, CommandEndIndex);

        //Serial.println("Command String is "+commandString);
        tempVar = commandString.indexOf(' ');

        if (tempVar >= 1)
        {
          // then there is at least one character before thespace
          commandStringStartText = commandString.substring(0, tempVar);
          // Now we need tounderstand if there is other whitespace in the remaining string
          //use inputString and tempVar again since we want to conserve memory

          inputString = commandString.substring(tempVar + 1);

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


        }
        else
        {
          // no space so there is no numbers to be parsed following the StartText
          commandStringStartText = commandString;
          // assign them to a negative number although we will not be using
          parse_no1 = -1;
          parse_no2 = -1;
        }
      }


#if DEBUGFLAG
      // Print the text
      Serial.println(" Command Text is " + commandStringStartText);
      Serial.print("Numbers are ");
      Serial.print(parse_no1);
      Serial.print(" and ");
      Serial.println(parse_no2);
#endif


      /* Comand database START
      UP  : Move UP
      DN  : Move DOWN
      LT  : Move LEFT
      RT  : Move RIGHT
      BX? : Send BX Value
      BY? : Send BY Value
      BZ? : Send BZ Value
      BXYZ? :Send BX"\t"BY"\t"BZ"\n"
      ACCX? : Send ACCX Value
      ACCY? : Send ACCY Value
      ACCXY? : Send ACCX"\t"ACCY"\n" Value
      ACCXLP? : Send ACCX afer Low Pass filtering Nsamples_LP times
      ACCYLP? : Send ACCY afer Low Pass filtering Nsamples_LP times
      STOPUD  : Stop Up-Down Motor of the Selected Mirror
      STOPLR  : Stop Left-Right Mirror of the Selected Mirror
      STOP   : Stop current selected motor
      STOPALL : Stop all the motors
      
      */
      // NOwow perform the command after converting it to Uppercase
      commandStringStartText.toUpperCase();

#if DEBUGFLAG
      Serial.print("Command uppercase is ");
      Serial.println( commandStringStartText);
#endif
      if (commandStringStartText == "UP")
      {

        MoveUpSelectedMirror ();
      }
      else if  (commandStringStartText == "DN")
      {
        MoveDownSelectedMirror ();
      }
  else if  (commandStringStartText == "LT")
      {
        MoveLeftSelectedMirror ();
      }
        else if  (commandStringStartText == "RT")
      {
        MoveRightSelectedMirror ();
      }
else if (commandStringStartText == "BX?")
{
    Serial.println(HMC5803L_Read(X));
}

else if (commandStringStartText == "BY?")
{
    Serial.println(HMC5803L_Read(Y));
}

else if (commandStringStartText == "BZ?")
{
    Serial.println(HMC5803L_Read(Z));
}

else if (commandStringStartText == "BXYZ?")
{
  Serial.print(HMC5803L_Read(X));
  Serial.print("\t");
  Serial.print(HMC5803L_Read(Y));
  Serial.print("\t");
  Serial.println(HMC5803L_Read(Z));
}  

else if (commandStringStartText == "ACCX?")
{
Serial.println(ReadAccelerationX());
}
else if (commandStringStartText == "ACCY?")
{
Serial.println(ReadAccelerationY());
}
else if (commandStringStartText == "ACCXY?")
{
Serial.print(ReadAccelerationX());
 Serial.print("\t");
 Serial.println(ReadAccelerationY());
}

else if (commandStringStartText == "ACCXLP?")
{
 Serial.println(ReadAccelerationX_LP());
}


else if (commandStringStartText == "ACCYLP?")
{
 Serial.println(ReadAccelerationY_LP());
}
else if (commandStringStartText == "STOPUD")
{
  // Turn off selected mirror's up down motor
  TurnOffSelectedMirror_UpDown();
}

else if (commandStringStartText == "STOPLR")
{
  // Turn off selected mirror's left-right motor
  TurnOffSelectedMirror_LeftRight();
}

else if (commandStringStartText == "STOP")
{
  // Turn off selected mirror
  TurnOffSelectedMirror();
}
else if (commandStringStartText == "STOPALL")
{
 // Turn off all the motors
  TurnOffAllMotors();
}


      else
      {
        // turn all the Motors off:
        TurnOffAllMotors();


      }

      // COMMAND DATABASE END




      // reset the String

      ResetInputString();

    }// if stringComplete






    // if any of the left, right, up, down vairables is true then the motor is active

    motorActive = LeftMotorState[selectedRow][selectedCol] || RightMotorState[selectedRow][selectedCol] ||
                  UpMotorState[selectedRow][selectedCol] || DownMotorState[selectedRow][selectedCol];




  } // if serial

  /*
    // print acceleration if motor is activel
    if (motorActive)
    {
      printAccelarationXY();
      printMagneticFieldXYZ();
      Serial.println();
    }


  */

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

  printAccelarationXY();
  printMagneticFieldXYZ();
  Serial.println();

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
bool UpdateSelectedMirror(const int parseTemp1,  const int parseTemp2)
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
    //printSelectMirror();
  }
  else
  {
    // Serial.println("Index out of range!");
    // printSelectMirror();
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

void printAccelarationXY (void) {

  // read pulse from x- and y-axes:
  pulseX = pulseIn(xPin, HIGH);
  pulseY = pulseIn(yPin, HIGH);

  // convert the pulse width into acceleration
  // accelerationX and accelerationY are in milli-g's:
  // earth's gravity is 1000 milli-g's, or 1g.
  accelerationX = ((pulseX / 10) - 500) * 8;
  accelerationY = ((pulseY / 10) - 500) * 8;

  /*
    // print the acceleration
    Serial.print(accelerationX);
    // print a tab character:
    Serial.print("\t");
    Serial.print(accelerationY);
    Serial.print("\t");
  */

}

// Read AccelerationX
int ReadAccelerationX()

{
  // read pulse from x- and y-axes:
  pulseX = pulseIn(xPin, HIGH);


  // convert the pulse width into acceleration
  // accelerationX and accelerationY are in milli-g's:
  // earth's gravity is 1000 milli-g's, or 1g.

  accelerationX = ((pulseX / 10) - 500) * 8;
  return  accelerationX;


}


// Read AccelerationY
int ReadAccelerationY()

{
  // read pulse from x- and y-axes:
  pulseY = pulseIn(yPin, HIGH);


  // convert the pulse width into acceleration
  // accelerationX and accelerationY are in milli-g's:
  // earth's gravity is 1000 milli-g's, or 1g.

  accelerationY = ((pulseY / 10) - 500) * 8;
  return  accelerationY;


}


// Read AccelerationX after low pass filtering with amoving average of Nsamples_LP

int ReadAccelerationX_LP()

{
  // initialize the integrator
  LowPassedData=0L;
  for (tempVar=0;tempVar<Nsamples_LP;tempVar++)
  {
  //pulseX=ReadAccelerationX();
 // Serial.print("Input Data is");
  // Serial.print(pulseX);
  
   LowPassedData+= (alpha_LP*((long)ReadAccelerationX()))>>15;
   //LowPassedData+= (alpha_LP*((long)pulseX))>>15;
    
   /* 
    Serial.print(" alpha_LP is ");
    Serial.print(alpha_LP);
    
    Serial.print(" LpwPassedData is ");
    Serial.println(LowPassedData);
    
    */
    // Without this delay , the acceleration readings come with an offset
    delay(DELAY_LP_ACCELERATION);
  }
    return  (int)LowPassedData;

}





// Read AccelerationY after low pass filtering with amoving average of Nsamples_LP

int ReadAccelerationY_LP()

{
  // initialize the integrator
  LowPassedData=0L;
  for (tempVar=0;tempVar<Nsamples_LP;tempVar++)
  {
  
   LowPassedData+= (alpha_LP*((long)ReadAccelerationY()))>>15;

    // Without this delay , the acceleration readings come with an offset
    delay(DELAY_LP_ACCELERATION);
  }
    return  (int)LowPassedData;

}


// move to a target tilt
void  movetoTilt(const int targetTilt)
{
  Serial.println("Moving!");
  currentTiltReading = ReadAccelerationX();
  // If there is a key entry d, get out of the loop


  while (abs(currentTiltReading - targetTilt) >= TILTEPSILON)
    // while ((abs(currentTiltReading - targetTilt) >= TILTEPSILON) && !Serial.available())
  {
    currentTiltReading = ReadAccelerationX();
    if (currentTiltReading > targetTilt)
      // move up
      MoveUpSelectedMirror();

    else
      // move down
      MoveDownSelectedMirror();


    /*
        // print state

        printAccelarationXY();
        printMagneticFieldXYZ();
        Serial.println( );

    */
  }



  // Turn off since we are done or another input is entered
  TurnOffSelectedMirror();



  //Serial.print("Precision: Current: ");
  //Serial.print(currentTiltReading);
  //Serial.print(" Target: ");
  //Serial.println(targetTilt);
}






// moves up selected motor
void MoveUpSelectedMirror (void) {

  //assert up motor
  UpMotorState[selectedRow][selectedCol] = true;

  //make sure you do not excite the Down at the same time
  // if the Up motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(UpMotorState[selectedRow][selectedCol],
                    &DownMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][3]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][2], UpMotorState[selectedRow][selectedCol]);


}



// moves down selected motor
void MoveDownSelectedMirror (void) {

  //assert down motor
  DownMotorState[selectedRow][selectedCol] = true;

  //make sure you do not excite the Up at the same time
  // if the Down motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(DownMotorState[selectedRow][selectedCol],
                    &UpMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][2]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][3], DownMotorState[selectedRow][selectedCol]);

}



// moves left selected motor
void MoveLeftSelectedMirror (void) {

  //assert left motor
  LeftMotorState[selectedRow][selectedCol] = true;

  //make sure you do not excite the Left at the same time
  // if the Right motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(LeftMotorState[selectedRow][selectedCol],
                    &RightMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][1]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][0], LeftMotorState[selectedRow][selectedCol]);

}




// moves Right selected motor
void MoveRightSelectedMirror (void) {

  //assert right motor
  RightMotorState[selectedRow][selectedCol] = true;

  //make sure you do not excite the Right at the same time
  // if the Left motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(RightMotorState[selectedRow][selectedCol],
                    &LeftMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][0]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][1], RightMotorState[selectedRow][selectedCol]);

}



void TurnOffSelectedMirror()
{

  LeftMotorState[selectedRow][selectedCol] = LOW;
  RightMotorState[selectedRow][selectedCol] = LOW;
  UpMotorState[selectedRow][selectedCol] = LOW;
  DownMotorState[selectedRow][selectedCol] = LOW;

  for (int motorIndex = 0; motorIndex < 4 ; motorIndex++)
  {
    digitalWrite(MotorPinList[selectedRow][selectedCol][motorIndex], LOW);
  }


}


// Turn -Off Selected Mirror's Up-Down Mirror only

void TurnOffSelectedMirror_UpDown()
{


  UpMotorState[selectedRow][selectedCol] = LOW;
  DownMotorState[selectedRow][selectedCol] = LOW;

  for (int motorIndex = 2; motorIndex < 4 ; motorIndex++)
  {
    digitalWrite(MotorPinList[selectedRow][selectedCol][motorIndex], LOW);
  }


}


// Turn -Off Selected Mirror's Left-Right Mirror only

void TurnOffSelectedMirror_LeftRight()
{


  LeftMotorState[selectedRow][selectedCol] = LOW;
  RightMotorState[selectedRow][selectedCol] = LOW;

  for (int motorIndex = 0; motorIndex < 2 ; motorIndex++)
  {
    digitalWrite(MotorPinList[selectedRow][selectedCol][motorIndex], LOW);
  }


}



// This function will toggle the Left motor of the selected motor

void ToggleLeftSelectedMirror(void)
{
  //toggle left motor
  LeftMotorState[selectedRow][selectedCol] = !LeftMotorState[selectedRow][selectedCol];

  //make sure you do not excite the Right at the same time
  // if the left motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(LeftMotorState[selectedRow][selectedCol],
                    &RightMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][1]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][0], LeftMotorState[selectedRow][selectedCol]);
  // print_state(selectedRow, selectedCol);

}



// This function will toggle the Right motor of the selected mirror

void ToggleRightSelectedMirror(void)
{
  //toggle right motor
  RightMotorState[selectedRow][selectedCol] = !RightMotorState[selectedRow][selectedCol];

  //make sure you do not excite the left at the same time
  // if the right motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(RightMotorState[selectedRow][selectedCol],
                    &LeftMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][0]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][1], RightMotorState[selectedRow][selectedCol]);
  //print_state(selectedRow, selectedCol);
}




// This function will toggle the Up motor of the selected mirror

void ToggleUpSelectedMirror(void)
{
  //toggle up motor
  UpMotorState[selectedRow][selectedCol] = !UpMotorState[selectedRow][selectedCol];

  //make sure you do not excite the Down at the same time
  // if the Up motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(UpMotorState[selectedRow][selectedCol],
                    &DownMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][3]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][2], UpMotorState[selectedRow][selectedCol]);

  // print_state(selectedRow, selectedCol);

}



// This function will toggle the Down motor of the selected mirror

void ToggleDownSelectedMirror(void)
{
  //toggle down motor
  DownMotorState[selectedRow][selectedCol] = !DownMotorState[selectedRow][selectedCol];

  //make sure you do not excite the Up at the same time
  // if the Down motor is on.
  // Running this first to avoid excitement of two motors at the same time.
  update_motor_pair(DownMotorState[selectedRow][selectedCol],
                    &UpMotorState [selectedRow][selectedCol],
                    MotorPinList[selectedRow][selectedCol][2]);
  digitalWrite(MotorPinList[selectedRow][selectedCol][3], DownMotorState[selectedRow][selectedCol]);
  //print_state(selectedRow, selectedCol);
}




/* This function will initialise the module and only needs to be run once
   after the module is first powered up or reset */
void Init_HMC5803L(void)
{
  /* Set the module to 8x averaging and 15Hz measurement rate */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(0x00);
  Wire.write(0x70);

  /* Set a gain of 1 - 0x20 corresponds to range +-1.3Ga*/
  /* Set a gain of 5 - 0xA0 corresponds to range +-1.3Ga*/
  Wire.write(0x01);
  Wire.write(0xA0);
  Wire.endTransmission();
}


/* This function will read once from one of the 3 axis data registers
and return the 16 bit signed result. */
int HMC5803L_Read(byte Axis)
{
  int Result;

  /* Initiate a single measurement */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(0x02);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(6);

  /* Move modules the resiger pointer to one of the axis data registers */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(Axis);
  Wire.endTransmission();




  /* Read the data from registers (there are two 8 bit registers for each axis) */
  Wire.requestFrom(HMC5803L_Address, 2);


  Result = Wire.read() << 8;
  Result |= Wire.read();



  return Result;
}



// This function prints the three componets of the magnetic field
void printMagneticFieldXYZ(void) {
  /* Read each sensor axis data and output to the serial port */

  Serial.print(HMC5803L_Read(X));
  Serial.print("\t");
  Serial.print(HMC5803L_Read(Y));
  Serial.print("\t");
  Serial.print(HMC5803L_Read(Z));


}


//reset input string for next parsing
void ResetInputString() {

  //Serial.println("Reseting Strings");
  // reset the String
  stringComplete = false;
  inputString = "";
  commandStringStartText = "";
  commandString = "";
  parse_no1 = 0;
  parse_no2 = 0;

}
