/*
   Memsic2125
   
   Read the Memsic 2125 two-axis accelerometer.  Converts the
   pulses output by the 2125 into milli-g's (1/1000 of earth's
   gravity) and prints them over the serial connection to the
   computer.
   
   The circuit:
    * X output of accelerometer to digital pin 2
    * Y output of accelerometer to digital pin 3
    * +V of accelerometer to +5V
    * GND of accelerometer to ground
  
   http://www.arduino.cc/en/Tutorial/Memsic2125
   
   created 6 Nov 2008
   by David A. Mellis
   modified 30 Aug 2011
   by Tom Igoe
   
   This example code is in the public domain.

 */


int pin = 13;
volatile int state = LOW;

void setup()
{
  pinMode(pin, OUTPUT);
  attachInterrupt(0, blink, RISING);
}

void loop()
{
  digitalWrite(pin, state);
}

void blink()
{
  state = !state;
}


