/**************************************************************************

    This sketch is is written for the Arduino Uno or Nano.  The Nano will 
    be installed between an FrSky S6r stabilized receiver, and a flying 
    wing's servos and ESCs. Its purpos is to perform advanced Elevon, 
    Differential Thrust (DT), and Thrust Vectoring (TV) mixing.

    For the newest version, visit:  https://github.com/jamieFL/vectoredair
    For set-up instructions:        http://www.vectoredair.com

   SERVO & ESC OUTPUTS
   Pin 2, Left Aileron Servo
   Pin 3, Left Thrust Vectoring Servo
   Pin 4, Left Electronic Speed Controller (ESC)
   Pin 5, Right Electonic Speed Controller (ESC)
   Pin 6, Right Thrust Vectoring Servo
   PIN 7, Right Aileron Servo   

   RECEIVER INPUTS
   Pin  8, PCINT0, Channel 1, Throttle
   Pin  9, PCINT1, Channel 2, Aileron
   Pin 10, PCINT2, Channel 3, Elevator
   Pin 11, PCINT3, Channel 4, Rudder
   Pin 12, PCINT4, Channel 5, Aux Channel

***************************************************************************

   INCLUDED LIBRARIES
   eRCaGuy_Timer2_Counter  http://tiny.cc/eRCaGuy_Timer2_Counter

   Note that the Timer 2 Counter Library has a precision of 0.5us.  To 
   maximze precision, and minimize servo jitter, all calculations are 
   performed in 0.5us counts, not micros.  That is why rx INPUTS and
   channel mixing calculations seem to be doubled.  To calculate micros, 
   divide any count value two.

   A servo needs a signal from about 1000us to 2000us where 1500us is
   center.  1000us is equivalent to 2000 counts and 2000us is equivalent
   to 4000 counts.  To convert counts to micros, divide by two.

    Counts = timer2.get_count() which has a precision of 0.5us.
    Micros = timer2.get_count()/2 which has a precision of 1us.


***************************************************************************

    Copyright (C) 2017 - Jim Lander (jamieFL)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/

**************************************************************************/

// *****  Included Libraries  *****
#include <eRCaGuy_Timer2_Counter.h>

// *****  Constants  *****
//#define DEBUG                               // Enable for Debugging Purposes 
//#define Serial_SPEED 115200                 // Comment Out For Use

// Receiver Input
#define RX_TOTAL_CHANNELS 5
#define RX_ENDPOINT_HIGH  4040
#define RX_ENDPOINT_LOW   1960

#define rxThr 0
#define rxAil 1
#define rxEle 2
#define rxRud 3
#define rxAux 4

#define rxThr_PIN 8
#define rxAil_PIN 9
#define rxEle_PIN 10
#define rxRud_PIN 11
#define rxAux_PIN 12

// Channel Mixing
#define dtLowRate  0.25
#define dtHighRate 1.0

// Servo Output
#define SERVO_TOTAL_CHANNELS 6
#define SERVO_PRECISION 8                   // Divide by 2 for us.
#define OUTPUT_LOOP_TIMER 39990             // Used my Ossilloscope to Calculate This to
                                            // match the frame width output by my receiver.
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
unsigned long rxTimer[RX_TOTAL_CHANNELS];   // Receiver Input Interrupt - count when Pin went HIGH
unsigned int rxPulse[RX_TOTAL_CHANNELS];    // Current Receiver Count

// Servo Output
unsigned int ltAilCount, ltTVCount, ltESCCount, rtESCCount, rtTVCount, rtAilCount;
unsigned long ltAilTime, ltTVTime, ltESCTime, rtESCTime, rtTVTime, rtAilTime, tStart;

// *****     Channel Mixing     *****
double reverse(double val) {                                 //  2000 - 4000 becomes 4000 - 2000
  return (val - 3000) * -1 + 3000;
}

double add(double val1, double val2) {                       // Combines Two Channels
  double out = (val1 - 3000 + val2 - 3000) + 3000;
  return constrain(out, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
}

void mixElevon(void) {                                       // Elevon Mixing of Aileron & Elevator Channels
  ltAilCount = add(rxPulse[rxAil], rxPulse[rxEle]);
  rtAilCount = add(rxPulse[rxAil], reverse(rxPulse[rxEle]));
}

void mixThrottle(float DTRate) {                             // Differential Thrust Calculations
  float yaw_pct, thr_pct;
  double upMod, dwnMod;
  uint32_t ltESCTmp, rtESCTmp;

  yaw_pct = ((float)rxPulse[rxRud] - 3000) / (float)1000;    // Value Between -1 and 1: Left to Right, 0 is Center
  if (yaw_pct < (float)-1.0) { yaw_pct = (float)-1.0; }      // Constrain between -1.0 and 1.0
    else if (yaw_pct > (float)1.0) { yaw_pct = (float)1.0; }
    
  thr_pct = ((float)rxPulse[rxThr] - 2000) / (float)2000;    // Value Between  0 and 1: No to Full Throttle
  if (thr_pct < (float)0.0) { yaw_pct = (float)0.0; }        // Constrain between 0.0 and 1.0
    else if (thr_pct > (float)1.0) { thr_pct = (float)1.0; }  

  dwnMod  = abs(yaw_pct) * (rxPulse[rxThr] - 2000) * DTRate; // Calculate Rudder Effect Constrained by Throttle
  upMod   = dwnMod * float(1-thr_pct);                       // Increas Modifier has less effect as throttle increases.

  if (yaw_pct < 0) {                                         // Rudder Left
    ltESCTmp = rxPulse[rxThr] - dwnMod;
    ltESCCount = constrain(ltESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
    rtESCTmp = rxPulse[rxThr] + upMod;
    rtESCCount = constrain(rtESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
  }
  else {                                                     // Rudder Right
    ltESCTmp = rxPulse[rxThr] + upMod;
    ltESCCount = constrain(ltESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
    rtESCTmp = rxPulse[rxThr] - dwnMod;
    rtESCCount = constrain(rtESCTmp, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
  }
}

void mixChannels(unsigned int Mode) {
  
    mixElevon();                                               // Elevon Mixing: ON
  if (Mode < 2500) {                                         // MODE 1
    rtTVCount = 3000;                                        // Thrust Vecoting: OFF
    ltTVCount = 3000;
    rtESCCount = rxPulse[rxThr];                             // Differential Thrust: OFF
    ltESCCount = rxPulse[rxThr];
  }
  else if (Mode > 3500) {                                    // MODE 3
    rtTVCount = rtAilCount;                                  // Thrust Vectoring: ON
    ltTVCount = ltAilCount;
    mixThrottle(dtHighRate);                                 // Differential Thrust: ON at HIGHT Rate
  }
  else {                                                     // MODE 2
    rtTVCount = 3000;                                        // Thrust Vectoring: OFF
    ltTVCount = 3000;
    mixThrottle(dtLowRate);                                  // Differential Thrust: ON at LOW Rate
  }
}

//  *****     Setup Routine     *****
void setup(){

  #ifdef DEBUG
    Serial.begin(Serial_SPEED);
    Serial.flush();
    Serial.println(" ");
    Serial.println("     Copyright (C) 2017 - Jim Lander (jamieFL)");  
    Serial.println("     This program comes with ABSOLUTELY NO WARRANTY.  It is free software: you can redistribute it and/or modify ");
    Serial.println("     it under the terms of the GNU General Public License.  For details, see http://www.gnu.org/licenses/");
    Serial.println(" ");
//    delay(4000)                                              // Pause 4 Seconds
  #endif

  // disable timer0 overflow interrupt
  TIMSK0 &= ~_BV(TOIE0);                                     //  Delay() and other standard timer functions are now disabled

  // Configure Servo Output Pins 2-7
  DDRD |= B11111100;

  // Setup Interrupts on Pins 8-12
  PCICR  |= (1 << PCIE0);                                    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  PCMSK0 |= (1 << PCINT4);  

  timer2.setup();                                            // Initialize the timer 2 library
  tStart = timer2.get_count();                               // Initialize Loop Timer

}

//  *****     Main Loop     *****
void loop(){
  static unsigned long tTime;

  // *****     Servo Mixing     *****
  mixChannels(rxPulse[rxAux]);                               // Perform Elevon, Thrust Vectoring, Differential Thrust Mixing

  // *****     Display Receiver INPUT and Servo OUTPUT     *****
  // When this code is active, it will interfere with servo movement.
  // Enable only the output you need for debugging.
  #ifdef DEBUG
    Serial.println("");
//    Serial.print("  Thr: "); Serial.print(rxPulse[rxThr]);   // Print Receiver INPUTS
    Serial.print("  Ail: "); Serial.print(rxPulse[rxAil]);
//    Serial.print("  Ele: "); Serial.print(rxPulse[rxEle]);
//    Serial.print("  Rud: "); Serial.print(rxPulse[rxRud]);
//    Serial.print("  Aux: "); Serial.print(rxPulse[rxAux]);
    Serial.print("    ltAl: "); Serial.print(ltAilCount);    // Print Servo OUTPUTS
    Serial.print("  rtAl: "); Serial.print(rtAilCount);
//    Serial.print("      ltTV: "); Serial.print(ltTVCount);
//    Serial.print("  rtTV: "); Serial.print(rtTVCount);
//    Serial.print("      lESC: "); Serial.print(ltESCCount);
//    Serial.print("  rESC: "); Serial.print(rtESCCount);  
  #endif

  // *****     Servo Output     *****

  // This timer loops until 4000us have passed since the 
  // beginning of the last servo output itteration.  It
  // accounts for the time taken for servo mixing and 
  //careful utilization of serial output.
  while(timer2.get_count() - tStart <= OUTPUT_LOOP_TIMER){  
    //    Serial.print(".");
  }
  tStart += OUTPUT_LOOP_TIMER;                               // increment start count for next itteration

  // This portion of code begins Servo Output.  It first sets 
  // all output pins to HIGH and then calcuates the time at 
  // which each pin will be set back to LOW.  Up to this point 
  // all calculation have been performed in Counts, which as 
  // previously stated, have a precision of 0.5us.  Such 
  //precision output to the servos may cause servo chatter because 
  // the signal changes 1-2us per iteration.  In integer math, 
  // by first dividing and then multiplying the count value by 
  // SERVO_PRECISION, the servo output timer's precision is 
  // reduced to SERVO_PRECISION/2 us.  I suggest a value of 4-8
  // which will give output precision of +/-(2 to 4)us.
  PORTD |= B11111100;
  ltAilTime = ((ltAilCount / SERVO_PRECISION) * SERVO_PRECISION) + tStart;
  ltTVTime  = ((ltTVCount  / SERVO_PRECISION) * SERVO_PRECISION) + tStart;
  ltESCTime = ((ltESCCount / SERVO_PRECISION) * SERVO_PRECISION) + tStart; 
  rtESCTime = ((rtESCCount / SERVO_PRECISION) * SERVO_PRECISION) + tStart;
  rtTVTime  = ((rtTVCount  / SERVO_PRECISION) * SERVO_PRECISION) + tStart;
  rtAilTime = ((rtAilCount / SERVO_PRECISION) * SERVO_PRECISION) + tStart;

  // This portion of code loops until all servo output pins
  // have been set back to LOW.
//  noInterrupts();
  while(PORTD >= 3){
//    interrupts();
    tTime = timer2.get_count();
//    Serial.print("+");
//    noInterrupts();
    if(ltAilTime <= tTime) PORTD &= B11111011;
    if(ltTVTime  <= tTime) PORTD &= B11110111;
    if(ltESCTime <= tTime) PORTD &= B11101111;
    if(rtESCTime <= tTime) PORTD &= B11011111;
    if(rtTVTime  <= tTime) PORTD &= B10111111;
    if(rtAilTime <= tTime) PORTD &= B01111111;
  }
//  interrupts();

}

// *****     Interrupts     ***** 

// Interrupts For When Receiver Input Pins Change State
ISR(PCINT0_vect){
  // Throttle Interrupt
  if(rxLast[rxThr] == 0 && PINB & B00000001 ){               // Input Signal Changed
    rxLast[rxThr] = 1;                                       // Input State HIGH
    rxTimer[rxThr] = timer2.get_count();                     // Remember Time Went High
  }
  else if(rxLast[rxThr] == 1 && !(PINB & B00000001)){        // Input Singal Changed
    rxLast[rxThr] = 0;                                       // Input State LOW
    rxPulse[rxThr] = (timer2.get_count() - rxTimer[rxThr]);  // Count Width is Time LOW - Time HIGH
  }
  // Aileron Interrupt
  if(rxLast[rxAil] == 0 && PINB & B00000010 ){
    rxLast[rxAil] = 1;
    rxTimer[rxAil] = timer2.get_count();
  }
  else if(rxLast[rxAil] == 1 && !(PINB & B00000010)){
    rxLast[rxAil] = 0;
    rxPulse[rxAil] = (timer2.get_count() - rxTimer[rxAil]);
  }
  // Elevator Interrupt
  if(rxLast[rxEle] == 0 && PINB & B00000100 ){
    rxLast[rxEle] = 1;
    rxTimer[rxEle] = timer2.get_count();
  }
  else if(rxLast[rxEle] == 1 && !(PINB & B00000100)){
    rxLast[rxEle] = 0;
    rxPulse[rxEle] = (timer2.get_count() - rxTimer[rxEle]);
  }
  // Rudder Interrupt
  if(rxLast[rxRud] == 0 && PINB & B00001000 ){
    rxLast[rxRud] = 1;
    rxTimer[rxRud] = timer2.get_count();
  }
  else if(rxLast[rxRud] == 1 && !(PINB & B00001000)){
    rxLast[rxRud] = 0;
    rxPulse[rxRud] = (timer2.get_count() - rxTimer[rxRud]);
  }  
  // Aux Channel Interrupt
  if(rxLast[rxAux] == 0 && PINB & B00010000 ){
    rxLast[rxAux] = 1;
    rxTimer[rxAux] = timer2.get_count();
  }
  else if(rxLast[rxAux] == 1 && !(PINB & B00010000)){
    rxLast[rxAux] = 0;
    rxPulse[rxAux] = (timer2.get_count() - rxTimer[rxAux]);
  }    
}


