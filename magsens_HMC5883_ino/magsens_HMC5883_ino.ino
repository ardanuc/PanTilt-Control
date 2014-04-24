/* FILE:    ARD_HMC5803L_GY273_Example
   DATE:    23/10/13
   VERSION: 0.1

This is an example of how to use the Hobby Components GY-273 module (HCMODU0036) which
uses a Honeywell HMC5883L 3-Axis Digital Compass IC. The IC uses an I2C interface to
communicate which is compatible with the standard Arduino Wire library.

This example demonstrates how to initialise and read the module in single shot
measurement mode. It will continually trigger single measurements and output the
results for the 3 axis to the serial port.


CONNECTIONS:

MODULE    ARDUINO
VCC       3.3V
GND       GND
SCL       A5
SDA       A4
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
REASON WHATSOEVER. */

/* Include the standard Wire library */
#include <Wire.h>

/* The I2C address of the module */
#define HMC5803L_Address 0x1E

/* Register address for the X Y and Z data */
#define X 3
#define Y 7
#define Z 5

void setup()
{
  Serial.begin(9600);
  /* Initialise the Wire library */
  Wire.begin();

  /* Initialise the module */
  Init_HMC5803L();

 
  /* Self Test */
  Serial.print("Self Test ");
  Serial.println (  HMC5803L_SelfTest(7) ? "Pass" : "Fail");
  

  delay (1000);


  /* Self Test */
  Serial.print("Self Test ");
  Serial.println (  HMC5803L_SelfTest(6) ? "Pass" : "Fail");


  
 delay (1000);


  /* Self Test */
  Serial.print("Self Test ");
  Serial.println (  HMC5803L_SelfTest(5) ? "Pass" : "Fail");


   delay (1000);


  /* Initialise the module */
  Init_HMC5803L();
}

void loop()
{
  delay(1000);
  /* Read each sensor axis data and output to the serial port */

  Serial.print(HMC5803L_Read(X));
  Serial.print(" ");
  Serial.print(HMC5803L_Read(Y));
  Serial.print(" ");
  Serial.println(HMC5803L_Read(Z));

  /* Wait a little before reading again */
  delay(200);
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


/* This function implements a self-test as described in the datasheet of HMC5833
and returns the optima gain_index as defined in the data sheet.gain _index is an integer between 0 to 7 (3 bits)*/


bool HMC5803L_SelfTest(int current_gain_index)
{
  bool passTestFlag = false;
  
  // current gain must be between 0 and 7
  constrain(current_gain_index, 0, 7);
  int Result [3]; // Self test vriables
  const int AXIS_List [] = {X, Y, Z};
  // GN2 GN1 GN0 from 0 to 7
  const int GAIN_List [] = {1370, 1090, 820, 660, 440, 390, 330 , 230};


  /* Set the module to 8x averaging and 15Hz measurement rate positive self test mode */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(0x00);
  Wire.write(0x71);



  /* Set a gain of 5 - 0xA0 corresponds to range +-1.3Ga*/
  /* Curent gain shifted left 5 times to get CRB7-CRB5*/
  Wire.write(0x01);
  Wire.write(current_gain_index << 5);

  
  
  /* Continuous measurement mode*/
  Wire.write(0x02);
  Wire.write(0x00);

  Wire.endTransmission();

  // Wait for 7 ms , datasheet recommends 6ms
  delay (6);



  /* Now read all the 3 axis data */

  for (int i = 0; i < 3; i++)
  {
    /* Move modules the resiger pointer to one of the axis data registers */
    Wire.beginTransmission(HMC5803L_Address);
    Wire.write(AXIS_List[i]);
    Wire.endTransmission();
   

  


    /* Read the data from registers (there are two 8 bit registers for each axis) */
    Wire.requestFrom(HMC5803L_Address, 2);
    Result[i] = Wire.read() << 8;
    Result[i] |= Wire.read();


    Serial.print("Self Test - ");
    Serial.print(i);
    Serial.print(" at gain ");
    Serial.print(current_gain_index);
    Serial.print(": ");
    Serial.println(Result[i]);


  }

  passTestFlag = true;
  for (int i = 0; i < 3; i++)
  {
    

    
    
    passTestFlag &= (Result[i]>=(GAIN_List [current_gain_index]*243L)/390) &&
                    (Result[i]<=(GAIN_List [current_gain_index] * 575L) / 390);
    
 

    if (!passTestFlag) break;
  }





  /* Set the module to 8x averaging and 15Hz measurement rate */
  Wire.beginTransmission(HMC5803L_Address);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

delay (62);
// dump the reading after self-test

 HMC5803L_Read(X);
delay(62);
 HMC5803L_Read(Y);
delay(62);
 HMC5803L_Read(Z);
delay(62);
  return passTestFlag;

}


