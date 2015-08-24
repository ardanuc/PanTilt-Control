/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example for Getting Started with nRF24L01+ radios. 
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two 
 * different nodes.  Put one of the nodes into 'transmit' mode by connecting 
 * with the serial monitor and sending a 'T'.  The ping node sends the current 
 * time to the pong node, which responds by sending the value back.  The ping 
 * node can then see how long the whole cycle took.
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 

RF24 radio(9,10);

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  
//

// The various roles supported by this sketch
typedef enum { remote= 1, actuator} role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "remote", "actuator"};

// The role of the current running sketch
role_e role = remote;

// input pin to control (INPUT_PULLUP)
//If LOW then it is an actuator node
const int RolePin=3;


const int LED_count=2;

// LEDs in the actuator node
const int LED_PIN_LIST[LED_count]={7,8};

//Switches in the remote
const int SWITCH_PIN_LIST[LED_count]={6,5};

#define SERIAL_DEBUG_CUSTOM


#ifdef SERIAL_DEBUG_CUSTOM
#define IF_SERIAL_DEBUG(x) {x;}
#else
#define IF_SERIAL_DEBUG(x) 
#endif 




// 
bool SWITCH_state[2] = {false,false};
bool LED_state[2] = {false,false};
//Sting to SEnd
char  messageSendString[30]="What the hell!";
char  messageReadString[30]="";

void setup(void)
{
  //
  // Print preamble
  //

  Serial.begin(57600);
  printf_begin();
  printf("\n\rRF24/examples/GettingStarted/\n\r");
  printf("ROLE: %s\n\r",role_friendly_name[role]);
  
 //declare pinmode to be an INPUT_PULLUP
  pinMode(RolePin, INPUT_PULLUP);
  
  bool rolePinValue=digitalRead(RolePin); 
  if (rolePinValue==HIGH)
  {
  role= remote;
  printf("\nThis is REMOTE.\n");
  // make the switch pins INPUT_PULLUP

 //declare pinmode to be an INPUT_PULLUP

  for (int switch_index=0; switch_index<LED_count;switch_index++)
  pinMode(SWITCH_PIN_LIST[switch_index], INPUT_PULLUP);
  

}
  else
 {
  role=actuator;
   printf("\nThis is ACTUATOR.\n");
  
    for (int LED_index=0; LED_index<LED_count; LED_index++)
    
    {
  pinMode(LED_PIN_LIST[LED_index], OUTPUT);
    }


 }
    
 
  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  if ( role == remote )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    // //
    // // Stop listening
    // //
   
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
    //
    // Start listening
    //
     
  }

  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
}

void loop(void)
{
  //
  // Remote role
  //

  if (role == remote)

  {



// First read the bits
    
    for (int LED_index=0; LED_index<LED_count; LED_index++)
    
    {
    SWITCH_state[LED_index]=digitalRead(SWITCH_PIN_LIST[LED_index]);
    IF_SERIAL_DEBUG(printf("LED %d state is %d.\n",LED_index, SWITCH_state[LED_index]));
    }
    
    
    uint8_t dataToSend = 0x00;

       for (int LED_index=0; LED_index<LED_count; LED_index++)
    
    {
    dataToSend|= (SWITCH_state[LED_index])<<LED_index;

    }

    IF_SERIAL_DEBUG(printf("%s: DAta to Send is decimal %d .\n",role_friendly_name[role],dataToSend))




  // First, stop listening so we can talk.
    radio.stopListening();

    // // Take the time, and send it.  This will block until complete
    // unsigned long time = millis();
    // printf("Now sending %lu...",time);
    // bool ok = radio.write( &time, sizeof(unsigned long) );
    
    // if (ok)
    //   printf("ok...");
    // else
    //   printf("failed.\n\r");



  


    // // Take the time, and send it.  This will block until complete
    //  time = millis();

     IF_SERIAL_DEBUG(printf("Now sending %x...",dataToSend));
     bool  ok = radio.write( &dataToSend, sizeof(uint8_t) );
    
    if (ok){
      IF_SERIAL_DEBUG( printf("ok..."));
    }
    else{
       IF_SERIAL_DEBUG(printf("failed.\n\r"));
}










    // Now, continue listening
    radio.startListening();

    // // Wait here until we get a response, or timeout (250ms)
    // unsigned long started_waiting_at = millis();
    // bool timeout = false;
    // while ( ! radio.available() && ! timeout )
    //   if (millis() - started_waiting_at > 200 )
    //     timeout = true;

    // // Describe the results
    // if ( timeout )
    // {
    //   printf("Failed, response timed out.\n\r");
    // }
    // else
    // {
    //   // Grab the response, compare, and send to debugging spew
    //   unsigned long got_time;
    //   radio.read( &got_time, sizeof(unsigned long) );

    //   // Spew it
    //   printf("Got response %lu, round-trip delay: %lu\n\r",got_time,millis()-got_time);
    // }



    // Try again 1s later
    delay(1000);



  }



  //
  // ACtuator role
  //

if ( role == actuator )
  {





// if there is data ready
    if ( radio.available() )
    {


  //     // Dump the payloads until we've gotten everything
  //     unsigned long got_time;
  //     bool done = false;
  //     while (!done)
  //     {
  //       // Fetch the payload, and see if this was the last one.
  //       done = radio.read( &got_time, sizeof(unsigned long) );

  //       // Spew it
  //       printf("Got payload %lu...",got_time);

  // // Delay just a little bit to let the other unit
  // // make the transition to receiver
  // delay(20);
  //     }



      // Dump the payloads until we've gotten everything
    uint8_t dataToReceive=9 ;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &dataToReceive, sizeof(uint8_t) );

        // Spew it
         IF_SERIAL_DEBUG(printf("Got payload %d...",dataToReceive));

  // Delay just a little bit to let the other unit
  // make the transition to receiver
  delay(20);
      }


 // assign the switch letters
    for (int LED_index=0; LED_index<LED_count; LED_index++)
    
    {
    LED_state[LED_index]=dataToReceive& _BV(LED_index);
    IF_SERIAL_DEBUG(printf("LED %d state is %d.\n",LED_index, LED_state[LED_index]));
    digitalWrite(LED_PIN_LIST[LED_index],LED_state[LED_index]);
    }

    /*

    

      // // First, stop listening so we can talk
      // radio.stopListening();

      //  snprintf(messageSendString, sizeof(messageSendString), "All good buddy");

      // // Send the final one back.
      // radio.write( messageSendString, sizeof(messageSendString) );
      // printf("Sent response.\n\r");

      // // Now, resume listening so we catch the next packets.
      // radio.startListening();
         
         */
          }

         
   }

  
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
