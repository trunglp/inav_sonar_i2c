/* 
   LPTRUNG   
   05/02/2018
   http://www.instructables.com/id/Asynchronously-Reading-the-HC-SR04-Sensor/

   A2 connect to pin Triger
   A3 connect to pin Echo
*/

#include "Wire.h"

#define I2C_SLAVE_ADDRESS 0x14
#define STATUS_OK 0
#define STATUS_OUT_OF_RANGE 1

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif


uint8_t i2c_regs[] =
{
    0, //status
    0, //older 8 of distance
    0, //younger 8 of distance
};
const byte reg_size = sizeof(i2c_regs);
volatile byte reg_position = 0;


//You may get an error when compiling that reads something like 'stray \342'
//This is an issue with copy+pasting out of instructable's website (they add invisible characters to text).
//I tried to remove them all and paste the code back in, but if the issue persists you may just need to retype the code by hand.

unsigned long pulseStartTime, echoStartTime, durationOfEcho, timeOfLastInterrupt, ledtime;
uint16_t distance;
boolean previousInterruptWasHigh = false;
boolean waitingForEcho = false;
boolean sendingPulse = false;

static boolean status_led = 0;

void setup(){
  DDRC |= B00000100; //Set port A2 to output
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT11); //PCINT11 corresponds to analog pin 3

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);  
}


void loop(){
  
  if(!waitingForEcho){   
   if(!sendingPulse){
        pulseStartTime = micros(); // record the start time of the pulse.
        PORTC |= B00000100; // use the PORTC register to set analog pin A2 HIGH.
        sendingPulse = true; // we don't want to execute this block of code twice in a row, so let's set this to true.
      }else if(pulseStartTime + 12 < micros()){ // if the start time + 12 microseconds is greater than the current micros() time
        PORTC &= B00000000; // use the PORTC register to set any currently HIGH ports to LOW.
        sendingPulse = false; // we're done sending the pulse, so we can set this to false.
        waitingForEcho = true; // now we want to wait for the echo to come back.
      }
  }

   uint32_t now = millis();
   if(ledtime+5000 < now) {    
      status_led = !status_led;
      digitalWrite(13, status_led ? HIGH : LOW);   // set the LED off
      ledtime = now + 1000;      
    }
    

    
      
  
  distance = ((uint16_t)durationOfEcho/2)/29.1;
  i2c_regs[0] = STATUS_OK;
  
  if(millis()-timeOfLastInterrupt > 20){
    //distance = -1;
    distance = 0; // lptrung fix 
    i2c_regs[0] = STATUS_OUT_OF_RANGE;
  }

  i2c_regs[1] = distance >> 8;
  i2c_regs[2] = distance & 0xFF;

  ////if distance = -1 => i2c_regs[1] = -1  va i2c_regs[2] = 4095
  ////if distance = 0 => i2c_regs[1] = 0  va i2c_regs[2] = 0
  // run test online C https://www.jdoodle.com/c-online-compiler
}
ISR(PCINT1_vect){  
  timeOfLastInterrupt = millis();
  if(PINC & B00001000 ){                                     // Is analog pin 3 high?
    if(!previousInterruptWasHigh){                         // If the last time this interrupt was run, the state was not HIGH
      echoStartTime = micros();                             // Mark the time this interrupt happened.
      previousInterruptWasHigh = true;                  // Make sure this block of code doesn't get run again next time
    }
  }else{ // Since the last if statement didn't get executed, analog pin 3 must've switched to LOW.
    durationOfEcho = micros() - echoStartTime; // Determine the duration that pin 3 was HIGH.
    waitingForEcho = false; // This is the boolean we used in loop(). We're no longer waiting for the echo.
    previousInterruptWasHigh = false;
  }
}


///////////////////////////////////////////////////////////////////////////////

void requestEvent()
{
  Wire.write(i2c_regs,3);
}

void receiveEvent(uint8_t howMany) {
    if (howMany < 1) {
        // Sanity-check
        return;
    }

    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    reg_position = Wire.read();

    howMany--;

    if (!howMany) {
        // This write was only to set the buffer for next read
        return;
    }
    
    // Everything above 1 byte is something we do not care, so just get it from bus as send to /dev/null
    while(howMany--) 
    {
      Wire.read();
    }
}

