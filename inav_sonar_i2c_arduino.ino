#include "WireMW.h"

#define USE_HCSR04
#define I2C_SLAVE_ADDRESS 0x14


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




#define PULSE_TIMEOUT (300 * 54) //this is an equivalent of 4 meters range
#define trigPin A3
#define ECHO_PIN A2
#define LED_PIN 13 
long duration, distance;
volatile uint8_t wakeCounter = 0;


#ifdef USE_HCSR04
  #define PULSE_TO_CM 58 //Multiplier pulse length to distance in [cm]
  #define MEASUREMENT_PERIOD_RATIO 5 // measurement rate = MEASUREMENT_PERIOD_RATIO * 16, 80ms in this case
  #define MAX_RANGE 300 //Range of 4 meters
#endif

#define MEASUREMENT_PERIOD_RATIO 5 // measurement rate = MEASUREMENT_PERIOD_RATIO * 16, 80ms in this case
  
#define STATUS_OK 0
#define STATUS_OUT_OF_RANGE 1
#define PULSE_TIMEOUT (MAX_RANGE * PULSE_TO_CM) //this is an equivalent of 4 meters range

//////////////////////////////////////////////////////////////////////////////////////
// I2C handlers
// Handler for requesting data
//
void requestEvent()
{ 
 Wire.write(i2c_regs,3); 
}


void receiveEvent(uint8_t bytesReceived) {
    if (bytesReceived < 1) {
        // Sanity-check
        return;
    }

    if (bytesReceived > TWI_RX_BUFFER_SIZE)
    {       
        return;
    }
    reg_position = Wire.read();
    bytesReceived--;
    if (!bytesReceived) {
        // This write was only to set the buffer for next read
        return;
    }
    
    // Everything above 1 byte is something we do not care, so just get it from bus as send to /dev/null
    while(bytesReceived--) 
    {
      Wire.read();
    }
}




void setup() {
  //Serial.begin(9600);
  delay(1000); //give the GPS receiver time to boot
  
  // Pin change interrupt control register - enables interrupt vectors
  PCICR  |= (1<<PCIE1); // Port C
  // Pin change mask registers decide which pins are enabled as triggers
  PCMSK1 |= (1<<PCINT10); // pin 2 PC2
  DDRC |= 0x08; //triggerpin PC3 as output

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

}


long microsecondsToCentimeters(long microseconds){
//  return microseconds / PULSE_TO_CM;
  return (microseconds * 34 / 100 / 2) / 10;
}

void loop() {

}


ISR(PCINT1_vect) {

 //if (wakeCounter == MEASUREMENT_PERIOD_RATIO) {
    PORTC &= ~(0x08);//PC3 low    
    delayMicroseconds(2);
    PORTC |= (0x08);//PC3 high 
     // wait 10 microseconds before turning off
    delayMicroseconds(10);
    PORTC &= ~(0x08);//PC3 low;

    long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);

    if (duration > 0) {
      i2c_regs[0] = STATUS_OK;
    } else {
      i2c_regs[0] = STATUS_OUT_OF_RANGE;
        //Serial.println("errir");
    }
    
    uint16_t cm = (uint16_t) microsecondsToCentimeters(duration);
    
    if (cm <10) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    i2c_regs[1] = cm >> 8;
    i2c_regs[2] = cm & 0xFF;
     //Serial.println(cm);
    wakeCounter = 0;
    
 //}


}
