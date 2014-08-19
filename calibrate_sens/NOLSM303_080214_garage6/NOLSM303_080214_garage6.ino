/*

- Revised on 07/28/2014
- Removed the Serial out during feedback MVTTR
- Added  the following commands
    MOTONAZ?: Send whether Azimuth motor is on or not
    MOTONEL?: Send whether Elevation motor is on or not
- Allowed multiple command execution when the feedback loops are executing
through MVTT1D and MVTT2D- 

-Revised on 07/26/2014
No more dropping the last 4 bit  of the accelerometer reading, to use the full
16 bit resolution
-Default settings are +-2g and 4g/2^16 =61micro g/LSB resolution
- Added an ID, which is by default set to PIXEL_ID parameter

-Revised on 07/19/2014
ADded precompiler string to make the LSM303 magsensor optional

#define ACCMAGSENS_CONNECTED 0  ---> LSM303 is not connected
#define ACCMAGSENS_CONNECTED 1  ---> LSM303 is connected


-Revised on 06/30/2014
Added commands to control a laser pointer


-Revision on 06/29/2014
Modified to  handle an arbitrary number of input arguments as part of the command string

Revision on :06/28/2014
-Corrected the bug when the interpreter accepted commands that did not
startwith the <CommandStartChar>

 Scanner Control
 Date: 06/20/2014



ARduino is used to control and poll the data, It only responds the commands when they
follow the following format

CommandStartChar='$';
CommandEndChar='!';
So the format of the command is


<CommandStartChar><CommandString><CommandEndChar>

Options for the CommandString are
SR<space><number> Eg: If  'SR 1' It will select Row 1
SR?               Poll for the selected row, and Arduino will return the Sected Row

Comman list is given later:
The e-compass module used is LSM303D from PoLOLU
LSM303D library is taken from "https://github.com/pololu/lsm303-arduino"



CONNECTIONS for UNO R3
Arduino      LSM303 board
-------------------------
     5V  ->  VIN
    GND  ->  GND
    SDA  ->  SDA
    SCL  ->  SCL

    for MOTOR-1const int AzimLeftPin_init = 2;
const int AzimRightPin_init = 3;
const int ElevUpPin_init = 4;
const int ElevDownPin_init = 5;

// Laser Pointer Pin-- MAke sure it does not collide with motor control pins
define LASER_POINTER_PIN 13

#define AZIM_POTENC_PIN A0
#define ELEV_POTENC_PIN A1


 CONNECTIONS for Micro R3
 Arduino      LSM
----------------
     5V  ->  VIN
    GND  ->  GND
    2  ->  SDA
    3  ->  SCL

   for MOTOR-1 digital pins
const int AzimLeftPin_init = 6;
const int AzimRightPin_init = 7;
const int ElevUpPin_init = 8;
const int ElevDownPin_init = 9;



 */


// Maximum length of the commands from the serial port including start and end characters
#define MAX_COMMAND_STRING_LENGTH 40
#define MAX_COMMAND_STRINGSTARTTEXT_LENGTH 10
#define MAX_NUMBER_ARGUMETS 4
#define DEBUGFLAG 0




// Flag to skip Low pass filter procesing to speed up the main feedback loop
#define PERFORM_LPF_PROCESSING 1
// Low pass moving average numberof samples
#define NSAMPLES_LP 5


// This is for different sensor calibrations
#define MAG_SENSOR_ID 2

#define LASER_POINTER_PIN 13

// Potentiometer Encoder Pins for Azimuth and Elevation Axis
#define AZIM_POTENC_PIN A0
#define ELEV_POTENC_PIN A1



// Flag to increase the ADC sampling call by 8 times, default prescaler is 128
// this reduces it to 16
// See the code in: http://forum.arduino.cc/index.php/topic,6549.0.html
#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbif
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
// This is for different sensor calibrations
# define MAG_SENSOR_ID 2l
# define PIXEL_ID  9

// Precompiler Flag to indicate if LSM303 is connected or not
#define ACCMAGSENS_CONNECTED 0

//Below is used as a safety for max-min 
#define MAXMINPOT_SAFETY_EPSILON 1

/* Include the standard Wire library */
#include <Wire.h>
/* Include LSM303 library */
#include <LSM303.h>
#include "specialfuncs.h"

//  Pixel ID is a readonly parameter for numberin the pixel
const int pixel_ID = PIXEL_ID;


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
long parse_no_array[MAX_NUMBER_ARGUMETS] = {0};
int tempVar;

//
int inByte;   // Byte read from serial port

// Command start and end characters. They smohould be set to different characters
const char CommandStartChar = '$';
const char CommandEndChar = '!';

// index of the CommandStartChar and CommandEndChar in the inputString
short CommandStartIndex;
short CommandEndIndex;

// number of input arguments
short CountParseNo = 0;

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




// Laser on-off state
bool LaserState = false;


// variable to keep track of whhether the motor is active or not


bool AZ_motorActive =false;
bool EL_motorActive =false;

short MotorPinList[nRowMotor][nColMotor][4] = {
  AzimLeftPin_init, AzimRightPin_init, ElevUpPin_init, ElevDownPin_init
};
    

// For report generation
char report[80];


LSM303 compass;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);

  // declare laser pin as the output, but make sure it does not coincide with mirror pins
  // initialize the digital pin as an output.
  pinMode(LASER_POINTER_PIN, OUTPUT);
  LaserState = false;
  //initially turn off the laser
  digitalWrite(LASER_POINTER_PIN, LaserState);

  /* Initialise the Wire library */
  Wire.begin();

#if ACCMAGSENS_CONNECTED==1
  compass.init();
  compass.enableDefault();
#endif


  /*

  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  /*
   min: { -1829,  -1831,  -1417}    max: { +2099,  +1865,  +2649}
  */

  //  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  //  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

#if ACCMAGSENS_CONNECTED==1

#if MAG_SENSOR_ID==1
  compass.m_min = (LSM303::vector<int16_t>) {
    -1829, -1831, -1417
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +2099, +1865, +2649
  };

#endif

#if MAG_SENSOR_ID==2
  compass.m_min = (LSM303::vector<int16_t>) {
    -1579, -1642, -2834
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +1488, +1240, +364
  };

#endif

#endif

  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.

  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.

  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use

  compass.heading((LSM303::vector<int>){0, 0, 1});

  to use the +Z axis as a reference.
  */


#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2) ;
  cbi(ADCSRA, ADPS1) ;
  cbi(ADCSRA, ADPS0) ;
#endif




  //Reserve 10 bytes for the input String
  inputString.reserve(MAX_COMMAND_STRING_LENGTH);

  commandString.reserve(MAX_COMMAND_STRING_LENGTH - 2);

  //Allow maximum 5 letter command String Start Text
  commandStringStartText.reserve(MAX_COMMAND_STRINGSTARTTEXT_LENGTH);

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



// add some safety for min-max
for (tempVar=0; tempVar<4;tempVar++)
{
AxDeadZone_EXT1[tempVar]=AxDeadZone_EXT1[tempVar]+MAXMINPOT_SAFETY_EPSILON;
AxDeadZone_EXT2[tempVar]=AxDeadZone_EXT2[tempVar]-MAXMINPOT_SAFETY_EPSILON;
}



}



void loop() {

  // read the command if there s any Serial bbyte available
  if (Serial.available() > 0) {

    // If command is available read it
    ReadSerialCommand();


    /* Comand database START
    UP     : Move UP
    DN     : Move DOWN
    LT     : Move LEFT
    RT     : Move RIGHT
    BX? : Send BX Value
    BY? : Send BY Value
    BZ? : Send BZ Value
    BXYZ? :Send BX"\t"BY"\t"BZ"\n"
    ACCX? : Send ACCX Value
    ACCY? : Send ACCY Value
    ACCZ? : Send ACCZ Value
    ACCXYZ? : Send ACCX"\t"ACCY"\t"ACCZ"\n" Value
    MAGHDG? : Send Magnetic Heding Angle in Degrees
    AZENC?  : Send Azim Potentiometer Encoder Raw Data
    ELENC?  : Send Elevation Potentiometer Encoder Raw Data
    PIXID?  : Send Pixel ID of the mirror
    MOTONAZ?: Send whether Azimuth motor is on or not
    MOTONEL?: Send whether Elevation motor is on or not
    LASON   : Turn the laser on
    LASOFF  : Turn the laser off
    LASTOG  : Toggle laser state
    STOPUD  : Stop Up-Down Motor of the Selected Mirror
    STOPLR  : Stop Left-Right Mirror of the Selected Mirror
    STOP   : Stop current selected motor
    STOPALL : Stop all the motors
    MVTT1D  :  Move to target 1D, takes two  parameters axis index and target value
    MVTT2D  :  Move to target 2D in AZIM and ELEC, takes two  target potentiometer values
    */
    // NOwow perform the command (Note that lowercase commands are
    // already converted to  Uppercase

    // If the start and end characters match and if the command is less than
    // the max alowed length
    if (stringComplete) {

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

#if ACCMAGSENS_CONNECTED==1
      else if (commandStringStartText == "BX?")
      {
        Serial.println(ReadMagneticFieldX());
      }

      else if (commandStringStartText == "BY?")
      {
        Serial.println(ReadMagneticFieldY());
      }

      else if (commandStringStartText == "BZ?")
      {
        Serial.println(ReadMagneticFieldZ());
      }

      else if (commandStringStartText == "BXYZ?")
      {
        PrintMagneticFieldXYZ();
      }

      else if (commandStringStartText == "ACCX?")
      {

        Serial.println(ReadAccelerationX());
      }
      else if (commandStringStartText == "ACCY?")
      {
        Serial.println(ReadAccelerationY());
      }
      else if (commandStringStartText == "ACCZ?")
      {
        Serial.println(ReadAccelerationZ());
      }
      else if (commandStringStartText == "ACCXYZ?")
      {
        PrintAccelerationXYZ();
      }

      else if (commandStringStartText == "MAGHDG?")
      {

        Serial.println(ReadMagneticHeading());
      }


#endif
      else if (commandStringStartText == "AZENC?")
      {

        Serial.println(analogRead(AZIM_POTENC_PIN));
      }
      else if (commandStringStartText == "ELENC?")
      {

        Serial.println(analogRead(ELEV_POTENC_PIN));
      }
      else if (commandStringStartText == "PIXID?")
      {

        Serial.println(pixel_ID);
      }
         else if (commandStringStartText == "MOTONAZ?")
      {

        Serial.println(AZ_motorActive); 
      }
        else if (commandStringStartText == "MOTONEL?")
      {

        Serial.println(EL_motorActive); 
      }
      else if (commandStringStartText == "LASON")
      {

        LaserUpdateState(true);
      }
      else if (commandStringStartText == "LASOFF")
      {

        LaserUpdateState(false);
      }
      else if (commandStringStartText == "LASTOG")
      {

        LaserUpdateState(!LaserState);
      }

      else if (commandStringStartText == "MVTT1D")
      {


        // Pointer to function to compare against to target value


        int   (*ReadFunctionPt_i) (void) = & ReadAccelerationY;
        long   (*ReadFunctionPt_l) (void) = & ReadMagneticHeading;

        // Pointer to function to action towards increasing the Value Read by ReadFunctionPt()
        void (*MovePositiveFunctionPt) (void) = &MoveRightSelectedMirror;

        // Pointer to function to action towards decreasing the Value Read by ReadFunctionPt()
        void (*MoveNegativeFunctionPt) (void) = &MoveLeftSelectedMirror;




        //Takes two parameters
        // One is the Axis label enum
        // and the other is the raw target value. Do range checking before
        //calling the llllllfunction

        if  (parse_no_array[1] != -1)

          switch (parse_no_array[0])
          {
#if ACCMAGSENS_CONNECTED==1
            case 0:   //Azim using magnetic heading
              {
                // Pointer to function to compare against to target value
                ReadFunctionPt_l = &ReadMagneticHeading;


                // Pointer to function to action towards increasing the Value Read by ReadFunctionPt()
                MovePositiveFunctionPt = &MoveRightSelectedMirror;

                // Pointer to function to action towards decreasing the Value Read by ReadFunctionPt()
                MoveNegativeFunctionPt = &MoveLeftSelectedMirror;


                // Azimuth axis, so run the angular feedback loop

                movetoTarget_1D_angular ( parse_no_array[0] , parse_no_array[1], ReadFunctionPt_l,
                                          MovePositiveFunctionPt, MoveNegativeFunctionPt);
                break;
              }
            case 1:   //Elevation using tilt Sensor
              {

                // Pointer to function to compare against to target value
                ReadFunctionPt_i = & ReadAccelerationY;

                // Pointer to function to action towards increasing the Value Read by ReadFunctionPt()
                MovePositiveFunctionPt = &MoveDownSelectedMirror;

                // Pointer to function to action towards decreasing the Value Read by ReadFunctionPt()
                MoveNegativeFunctionPt = &MoveUpSelectedMirror;

                // Elevation axis, so run the linear
                movetoTarget_1D_linear ( parse_no_array[0] , parse_no_array[1], ReadFunctionPt_i,
                                         MovePositiveFunctionPt, MoveNegativeFunctionPt);
                break;
              }

#endif
            case 2:   //Azim using Encoder
              {

                // Pointer to function to compare against to target value
                ReadFunctionPt_i = & ReadAZIM_PotentiometerEncoder;

                // Pointer to function to action towards increasing the Value Read by ReadFunctionPt()
                MovePositiveFunctionPt = &MoveRightSelectedMirror;

                // Pointer to function to action towards decreasing the Value Read by ReadFunctionPt()
                MoveNegativeFunctionPt = &MoveLeftSelectedMirror;

                // Elevation axis, so run the linear
                movetoTarget_1D_linear ( parse_no_array[0] , parse_no_array[1], ReadFunctionPt_i,
                                         MovePositiveFunctionPt, MoveNegativeFunctionPt);
                break;
              }
            case 3:   //Elev using Encoder
              {

                // Pointer to function to compare against to target value
                ReadFunctionPt_i = & ReadELEV_PotentiometerEncoder;

                // Pointer to function to action towards increasing the Value Read by ReadFunctionPt()
                MovePositiveFunctionPt = &MoveUpSelectedMirror;

                // Pointer to function to action towards decreasing the Value Read by ReadFunctionPt()
                MoveNegativeFunctionPt = &MoveDownSelectedMirror;

                // Elevation axis, so run the linear
                movetoTarget_1D_linear ( parse_no_array[0] , parse_no_array[1], ReadFunctionPt_i,
                                         MovePositiveFunctionPt, MoveNegativeFunctionPt);
                break;
              }


            default:
              {
                Serial.println("Wrong Axis!");
              }
          }//switch
      }

     // Move to target AZIM and ELEV pot reading
      else if (commandStringStartText == "MVTT2D")
      {

       if  ((parse_no_array[0] != -1) && (parse_no_array[1] != -1))
       {
         // first parameter is AZIM and the second is ELEV pot reading
         movetoTarget_2D_linear ((int)parse_no_array[0], (int) parse_no_array[1]);
       }
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





    // if any of the left, right, up, down vairables is true then make the corresponding 
    // motorACtive booleans true
    UpdateMotorActiveBooleans();
            



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

  //printAccelarationXYZ();
  //printMagneticFieldXYZ();
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


#if ACCMAGSENS_CONNECTED==1

//------ACCELERATION -----//

// Read AccelerationX
int ReadAccelerationX()
{
  compass.read();
  return compass.a.x ;
}

// Read AccelerationY
int ReadAccelerationY()
{
  compass.read();
  return compass.a.y ;
}

// Read AccelerationZ
int ReadAccelerationZ()
{
  compass.read();
  return compass.a.z ;
}


// Print Acceleration XYZ
int PrintAccelerationXYZ()
{
  compass.read();
  snprintf(report, sizeof(report), "%+6d\t%+6d\t%+6d",
           compass.a.x , compass.a.y , compass.a.z );
  Serial.println(report);
}

#endif

#if ACCMAGSENS_CONNECTED==1

//------MAGNATIC FIELD -----//


// Read MagneticFieldX
int ReadMagneticFieldX()
{
  compass.read();
  return compass.m.x;
}

// Read MagneticFieldY
int ReadMagneticFieldY()
{
  compass.read();
  return compass.m.y;
}

// Read MagneticFieldZ
int ReadMagneticFieldZ()
{
  compass.read();
  return compass.m.z;
}

// Read Magnetic Field XYZ
int PrintMagneticFieldXYZ()
{
  compass.read();
  snprintf(report, sizeof(report), "%+6d\t%+6d\t%+6d",
           compass.m.x, compass.m.y, compass.m.z);
  Serial.println(report);
}


// Read Heading as a multiple of 0.01 degrees
long ReadMagneticHeading()
{

  compass.read();
  return    100 * compass.heading();


}

#endif

/*------ LASER CONTROL ----------*/
/* ------- START ----------------*/

// Laser Update State updates the laser status with LaserStateInput
void LaserUpdateState(bool LaserStateInput)
{
  LaserState = LaserStateInput;
  digitalWrite(LASER_POINTER_PIN, LaserState);

}

/*------ LASER CONTROL ----------*/
/* ------- END ----------------*/


/*------ READ ENCODER  ----------*/
/* ------- START ----------------*/


// Read ADC output from the potentiometer reading of the azimuth
int ReadAZIM_PotentiometerEncoder()
{
  return analogRead(AZIM_POTENC_PIN);
}


// Read ADC output from the potentiometer reading of the azimuth
int ReadELEV_PotentiometerEncoder()
{
  return analogRead(ELEV_POTENC_PIN);
}


/*------ READ ENCODER ----------*/
/* ------- END ----------------*/

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


// Read Serial Command and store the core
// This script also parses up to MAX_NUMBER_ARGUMETS arguments.
// Uppercase of the Parsed command excluding start and end characters will
// be stored in the string commandStringStartText. Parsed numbers are stored in
// long parse_no_array[MAX_NUMBER_ARGUMETS]
void ReadSerialCommand(void) {
  inByte = Serial.read();

  // boolean of whether a valid character is entered or not, any number or letter and space characters are fine

  tempVar = ((inByte >= 'a') && (inByte <= 'z')) ||
            ((inByte >= '0') && (inByte <= '9')) ||
            ((inByte >= 'A') && (inByte <= 'Z')) ||
            (inByte == ' ') ||
            (inByte == CommandStartChar) ||
            (inByte == CommandEndChar) ||
            (inByte == '?') ||
            (inByte == '+') ||
            (inByte == '-') ;

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
  else  // when unrecognized characters are sent tuff all the motors
  {
    // turn all the Motors off:
    TurnOffAllMotors();
    // Serial.println("All Off!");

  }

  // If the start and end characters match and if the command is less than
  // the max alowed length
  if (stringComplete) {

    // Remove the whitespace at the beginning and at the end



    // Start Parsing
    /*This script assummes a command of the form
    <Text Command>[<space>< Number 1 >][<space><Number 2>]
    where <Number 1> and <Number 2> are optional. If they exist they are stored in
    variables parse_no_array[0] and parse_no_array[1], otherwise they are assigned a value of -
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

    if ((CommandEndIndex > CommandStartIndex + 1) && (CommandStartIndex != -1))
    {
      commandString = inputString.substring(CommandStartIndex + 1, CommandEndIndex);

      //trim the white space at the beginning and end
      commandString.trim();

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




        //parse the numbers with a while loop
        // If there is a whitespace or some non empty string , proviede that you have not parsed
        // all the numbes


        while (((tempVar >= 0) || (inputString.length() > 0)) && (CountParseNo < MAX_NUMBER_ARGUMETS))
        {


          parse_no_array[CountParseNo] = inputString.substring(0, tempVar).toInt();
          CountParseNo = CountParseNo + 1;



          //Get the remaining string after white space
          if (tempVar != -1)
            inputString = inputString.substring(tempVar + 1);
          else
            // since there is nothing left
            inputString = "";


          //MAke sure there is no more whitespace
          // caller might add more than one space
          inputString.trim();

          //find the whitespace index
          tempVar = inputString.indexOf(' ');
        }


      }
      else
      {
        // no space so there is no numbers to be parsed following the StartText
        commandStringStartText = commandString;
        // assign them to a negative number although we will not be using
        CountParseNo = 0;
        //set the rest of the parse no parameters to -1
        for (tempVar = 0; tempVar < MAX_NUMBER_ARGUMETS; tempVar++)
        {
          parse_no_array[tempVar] = -1;
        }

      }
    }


#if DEBUGFLAG
    // Print the text
    Serial.println(" Command Text is " + commandStringStartText);
    Serial.print("Numbers are ");
    for (tempVar = 0; tempVar < CountParseNo; tempVar++)
    {
      snprintf(report, sizeof(report), "No-%d: %d,\t",
               tempVar + 1, parse_no_array[tempVar] );
      Serial.print(report);
    }
    Serial.println();
#endif

    // COnvert the command to uppercase in case the user sent in lower case
    commandStringStartText.toUpperCase();

#if DEBUGFLAG
    Serial.print("Command uppercase is ");
    Serial.println( commandStringStartText);
#endif

  }// if stringComplete

} // void ReadSerialCommand(void)



// ExecuteMildCommandSet executes allows execution of certain commands that do 
// not significantly slow down the feedback loop refresh 

void ExecuteMildCommandSet(void){
// If the start and end characters match and if the command is less than
    // the max alowed length
    if (stringComplete) {
      if (commandStringStartText == "PIXID?")
      {

        Serial.println(pixel_ID);
      }
     else if (commandStringStartText == "MOTONAZ?")
      {

        Serial.println(AZ_motorActive); 
      }
        else if (commandStringStartText == "MOTONEL?")
      {

        Serial.println(EL_motorActive); 
      }
      // reset the String

      ResetInputString();

    }// if stringComplete     
    
}

// Update motor Active Booleans
  void  UpdateMotorActiveBooleans(void){
    
   AZ_motorActive= LeftMotorState[selectedRow][selectedCol] || RightMotorState[selectedRow][selectedCol];
   EL_motorActive=UpMotorState[selectedRow][selectedCol] || DownMotorState[selectedRow][selectedCol];
        
  }
  
  
// REad and Execute Mild Command SEt

void ReadandExecuteMildCommandSet(void) {

  // If command is available read it
  ReadSerialCommand();

  // Certain Commands can be executed within the feedback loop as long as they
  // do not significantly slow down the feedback loop. These commands are listed
  // in ExecuteMildCommandSet
  ExecuteMildCommandSet();

  //Update MotorActive Booleans
  UpdateMotorActiveBooleans();

}



//reset input string for next parsing
void ResetInputString() {

  //Serial.println("Reseting Strings");
  // reset the String
  stringComplete = false;
  inputString = "";
  commandStringStartText = "";
  commandString = "";
  CountParseNo = 0;
  for (tempVar = 0; tempVar < MAX_NUMBER_ARGUMETS; tempVar++)
  {
    parse_no_array[tempVar] = 0;
  }
}
