/**************************************************************************

   RECEIVER INPUTS
   Pin  8, PCINT0, Channel 1, Throttle
   Pin  9, PCINT1, Channel 2, Aileron
   Pin 10, PCINT2, Channel 3, Elevator
   Pin 11, PCINT3, Channel 4, Rudder
   Pin 12, PCINT4, Channel 5, Aux Channel

   SERVO & ESC OUTPUTS
   Pin 2, Left Aileron Servo
   Pin 3, Left Thrust Vectoring Servo
   Pin 4, Left Electronic Speed Controller (ESC)
   Pin 5, Right Electonic Speed Controller (ESC)
   Pin 6, Right Thrust Vectoring Servo
   PIN 7, Right Aileron Servo   


   INCLUDED LIBRARIES
   eRCaGuy_Timer2_Counter  http://tiny.cc/eRCaGuy_Timer2_Counter

***************************************************************************

    Copyright (C) 2017 - Jim Lander (jamieFL)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/

**************************************************************************/

// *****  Included Libraries  *****
#include <Servo.h>
#include <eRCaGuy_Timer2_Counter.h>

// *****  Constants  *****
#define SERIAL_SPEED 9600
#define SERIAL_DELAY 200

// Receiver Input
#define RX_TOTAL_CHANNELS  5
#define RX_ENDPOINT_HIGH 2020
#define RX_ENDPOINT_LOW 980

#define rxThr  0
#define rxAil  1
#define rxEle  2
#define rxRud  3
#define rxAux  4

#define rxThr_PIN  8
#define rxAil_PIN  9
#define rxEle_PIN  10
#define rxRud_PIN  11
#define rxAux_PIN  12

// Channel Mixing
#define dtLowRate 0.25
#define dtHighRate 1.0

// Servo Output
#define SERVO_TOTAL_CHANNELS 6

#define ltAil 0
#define ltTV  1
#define ltESC 2
#define rtESC 3
#define rtTV  4
#define rtAil 5

#define ltAil_PIN 2
#define ltTV_PIN  3
#define ltESC_PIN 4
#define rtESC_PIN 5
#define rtTV_PIN  6
#define rtAil_PIN 7

// *****  Variables  *****
// Receiver Input
byte rxLast[RX_TOTAL_CHANNELS];             // Receiver Input Interrupt - Last Interrupt State
unsigned long rxTimer[RX_TOTAL_CHANNELS];   // Receiver Input Interrupt - micros when pulse went high
int rxPulse[RX_TOTAL_CHANNELS];             // Current Receiver Pulse

// Servo Output
volatile uint32_t ltAilPulse, ltTVPulse, ltESCPulse, rtESCPulse, rtTVPulse, rtAilPulse;
unsigned long ltAilTime, ltTVTime, ltESCTime, rtESCTime, rtTVTime, rtAilTime;
volatile float DTRate;
unsigned long zero_timer, pulse_loop_timer;

Servo ServoArray[SERVO_TOTAL_CHANNELS];

// *****     Channel Mixing     *****
double reverse(double val) {
  return (val - 1500) * -1 + 1500;
}

double add(double val1, double val2) {
  double out = (val1 - 1500 + val2 - 1500) + 1500;
  return constrain(out, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
}

void mixElevon(void) {
  double tmpval = add(rxPulse[rxAil], rxPulse[rxEle]);
  ltAilPulse = constrain(tmpval, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
  tmpval = add(rxPulse[rxAil], reverse(rxPulse[rxEle]));  
  rtAilPulse = constrain(tmpval, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
}

void mixThrottle(void) {
  float yaw_pct, thr_pct;
  double upMod, dwnMod;
  uint32_t ltESCTmp, rtESCTmp;

  yaw_pct = ((float)rxPulse[rxRud] - 1500) / (float)500;    // Value Between -1 and 1: 0 for Center, -1 full left, 1 full right
  thr_pct = ((float)rxPulse[rxThr] - 1000) / (float)1000;     // Value Between 0 and 1: .5 for Mid Throttle
  
  if (yaw_pct < (float)-1.0) { yaw_pct = (float)-1.0; }       // Constrain values between -1.0 and 1.0
    else if (yaw_pct > (float)1.0) { yaw_pct = (float)1.0; }
  if (thr_pct < (float)0.0) { yaw_pct = (float)0.0; }         // Constrain values between 0.0 and 1.0
    else if (thr_pct > (float)1.0) { thr_pct = (float)1.0; }  

  dwnMod  = abs(yaw_pct) * (rxPulse[rxThr] - 1000) * DTRate;  // Calculate Rudder Pulse Size Constrained by Amount of Throttle
  upMod   = dwnMod * float(1-thr_pct);                        // Up Modifier has less of an effect as throttle increases.

  if (yaw_pct < 0) {  // Rudder Left
    ltESCTmp = reverse(rxPulse[rxThr] - dwnMod);
    ltESCPulse = constrain(ltESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
    rtESCTmp = rxPulse[rxThr] + upMod;
    rtESCPulse = constrain(rtESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
  }
  else {  // Rudder Right
    ltESCTmp = reverse(rxPulse[rxThr] + upMod);
    ltESCPulse = constrain(ltESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
    rtESCTmp = rxPulse[rxThr] - dwnMod;
    rtESCPulse = constrain(rtESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
  }
}

//  *****     Setup Routine     *****
void setup(){

  Serial.begin(SERIAL_SPEED);

  timer2.setup();           // Initialize Timer2_Counter; Note: since this messes up PWM outputs on 
                            // pins 3 & 11, as well as interferes with the tone() library, you can 
                            // always revert Timer2 back to normal by calling timer2.unsetup()

  
  // rx INPUT - Setup Interrupts on Pins 8-12
  PCICR  |= (1 << PCIE0);   // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  PCMSK0 |= (1 << PCINT4);  

  TIMSK0 &= ~_BV(TOIE0);    // disable timer0 overflow interrupt - NEEDED to reduce servo jitter

  // Servo OUTPUT
  ServoArray[ltAil].attach(ltAil_PIN, 900, 2100); //  Left Elevon
  ServoArray[ltTV].attach (ltTV_PIN, 900, 2100);  //  Left Thrust Vector Servo
  ServoArray[ltESC].attach(ltESC_PIN, 900, 2100); //  Left ESC
  ServoArray[rtESC].attach(rtESC_PIN, 900, 2100); //  Right ESC
  ServoArray[rtTV].attach (rtTV_PIN, 900, 2100);  //  Right Thrust Vector Servo
  ServoArray[rtAil].attach(rtAil_PIN, 900, 2100); //  Right Elevon  

}

//  *****     Main Loop     *****
void loop(){

if (rxPulse[rxAux] < 1250) {          // MODE 1
    mixElevon();                      // Elevon Mixing: ON
    rtTVPulse = 1500;                 // Thrust Vecoting: OFF
    ltTVPulse = 1500;
    rtESCPulse = rxPulse[rxThr];      // Differential Thrust: OFF
    ltESCPulse = reverse(rxPulse[rxThr]);
  }
  else if (rxPulse[rxAux] > 1750) {   // MODE 3
    mixElevon();                      // Elevon Mixing: ON
    rtTVPulse = rtAilPulse;           // Thrust Vectoring: ON
    ltTVPulse = ltAilPulse;
    DTRate = dtHighRate;              // Differential Thrust: ON at HIGHT Rate
    mixThrottle();
  }
  else {                              // MODE 2
    mixElevon();                      // Elevon Mixing: ON
    rtTVPulse = 1500;                 // Thrust Vectoring: OFF
    ltTVPulse = 1500;
    DTRate = dtLowRate;               // Differential Thrust: ON at LOW Rate
    mixThrottle();
  }

  ServoArray[ltAil].writeMicroseconds(ltAilPulse);
  ServoArray[ltTV].writeMicroseconds(ltTVPulse);
  ServoArray[ltESC].writeMicroseconds(ltESCPulse);
  ServoArray[rtESC].writeMicroseconds(rtESCPulse);
  ServoArray[rtTV].writeMicroseconds(rtTVPulse);
  ServoArray[rtAil].writeMicroseconds(rtAilPulse);

}

//Interrupt routines called when Receiver Input Pins change state
ISR(PCINT0_vect){
  // Throttle Interrupt
  if(rxLast[rxThr] == 0 && PINB & B00000001 ){
    rxLast[rxThr] = 1;
    rxTimer[rxThr] = timer2.get_count()/2;
  }
  else if(rxLast[rxThr] == 1 && !(PINB & B00000001)){
    rxLast[rxThr] = 0;
    rxPulse[rxThr] = timer2.get_count()/2 - rxTimer[rxThr];
  }
  // Aileron Interrupt
  if(rxLast[rxAil] == 0 && PINB & B00000010 ){
    rxLast[rxAil] = 1;
    rxTimer[rxAil] = timer2.get_count()/2;
  }
  else if(rxLast[rxAil] == 1 && !(PINB & B00000010)){
    rxLast[rxAil] = 0;
    rxPulse[rxAil] = timer2.get_count()/2 - rxTimer[rxAil];
  }
  // Elevator Interrupt
  if(rxLast[rxEle] == 0 && PINB & B00000100 ){
    rxLast[rxEle] = 1;
    rxTimer[rxEle] = timer2.get_count()/2;
  }
  else if(rxLast[rxEle] == 1 && !(PINB & B00000100)){
    rxLast[rxEle] = 0;
    rxPulse[rxEle] = timer2.get_count()/2 - rxTimer[rxEle];
  }
  // Rudder Interrupt
  if(rxLast[rxRud] == 0 && PINB & B00001000 ){
    rxLast[rxRud] = 1;
    rxTimer[rxRud] = timer2.get_count()/2;
  }
  else if(rxLast[rxRud] == 1 && !(PINB & B00001000)){
    rxLast[rxRud] = 0;
    rxPulse[rxRud] = timer2.get_count()/2 - rxTimer[rxRud];
  }  
  // Aux Channel Interrupt
  if(rxLast[rxAux] == 0 && PINB & B00010000 ){
    rxLast[rxAux] = 1;
    rxTimer[rxAux] = timer2.get_count()/2;
  }
  else if(rxLast[rxAux] == 1 && !(PINB & B00010000)){
    rxLast[rxAux] = 0;
    rxPulse[rxAux] = timer2.get_count()/2 - rxTimer[rxAux];
  }    
}
