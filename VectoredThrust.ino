/**************************************************************************
    This sketch is is designed for the Arduino Uno or Nano.  The Nano will 
    be installed between an FrSky S6r stabilized receiver, and a flying 
    wing's servos and ESCs. Its purpos is to perform Elevon, Differential 
    Thrust (DT), and Thrust Vectoring (TV) mixing.

   RECEIVER INPUTS
   Pin 8, Channel 1, Throttle
   Pin 9, Channel 2, Aileron
   Pin 10, Channel 3, Elevator
   Pin 11 Channel 4, Rudder
   Pin 12, Channel 5, Auxilary Channle 1 - Mode Switch
                     Mode 1, PWM < 1250, TV Off, DT @ 15%
                     Mode 2, PWM ~1500, TV On, DT @ 15%
                     Mode 3, PWM > 1750, TV On, DT 100%

   SERVO & ESC OUTPUTS
   Pin 2,  Left Aileron Servo
   Pin 3,  Left Thrust Vectoring Servo
   Pin 4, Left Electronic Speed Controller (ESC)
   Pin 5, Right Electonic Speed Controller (ESC)
   Pin 6, Right Thrust Vectoring Servo
   PIN 7, Right Aileron Servo

   INCLUDED LIBRARIES
   Enable Interrupt        http://tiny.cc/EnableInterrupt
   eRCaGuy_Timer2_Counter  http://tiny.cc/eRCaGuy_Timer2_Counter
   Servos                  http://tiny.cc/arduino-servo

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

#include <EnableInterrupt.h>
#include <Servo.h>
#include <eRCaGuy_Timer2_Counter.h>

#define SERIAL_SPEED 115200
#define LOOP_DELAY 2000     // If Servos attached, use small value

// *****     Debug Output - If defined, enable Serial Monitor     *****
// *****          EXPECT SERVO JITTER IN DEBUG MODE               *****
// #define DEBUG_SETUP         // Must be defined if any of the below are defined 
// #define DEBUG_RX            // Debug Output for Receiver Input
// #define DEBUG_SERVO         // Debug Output for Servo Output
// #define DEBUG_MIXING        // Debug Output for Signal Processing

// *****     rx INPUT     *****
#define RX_TOTAL_CHANNELS  5
#define RX_ENDPOINT_HIGH 2000
#define RX_ENDPOINT_LOW 1000

#define rxThr  0
#define rxAil  1
#define rxEle  2
#define rxRud  3
#define rxAux 4

#define rxThr_PIN  8
#define rxAil_PIN  9
#define rxEle_PIN  10
#define rxRud_PIN  11
#define rxAux_PIN 12

// *****     rx Servo Output     *****
#define SERVO_TOTAL_CHANNELS 6
#define dtLowRate 0.25
#define dtHighRate 1.0
#define SERVO_ENDPOINT_HIGH 2000
#define SERVO_ENDPOINT_LOW 1000

#define ltAil 0
#define ltTV  1
#define ltESC 2
#define rtESC 3
#define rtTV  4
#define rtAil 5

#define ltAil_PIN  2
#define ltTV_PIN   3
#define ltESC_PIN  4
#define rtESC_PIN  5
#define rtTV_PIN   6
#define rtAil_PIN  7

// *****     rx INPUT     *****
uint16_t rxPulse[RX_TOTAL_CHANNELS];
uint64_t rxPulseStart[RX_TOTAL_CHANNELS];
volatile uint16_t rxPulseTemp[RX_TOTAL_CHANNELS];

// *****     rx Servo Output     *****
volatile uint32_t ltAilPulse, ltTVPulse, ltESCPulse, rtESCPulse, rtTVPulse, rtAilPulse;
volatile float DTRate;

Servo ServoArray[SERVO_TOTAL_CHANNELS];

// *****     rx INPUTS     *****
void rxReadPulse() {
  noInterrupts();
  memcpy(rxPulse, (const void *)rxPulseTemp, sizeof(rxPulseTemp));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rxPulseStart[channel] = timer2.get_count()/2;
  } else {
    uint16_t rxPulsTotal = (uint16_t)((timer2.get_count()/2) - rxPulseStart[channel]);
    rxPulseTemp[channel] = constrain(rxPulsTotal, RX_ENDPOINT_LOW, RX_ENDPOINT_HIGH);
  }
}

void calc_Thr()  {
  calc_input(rxThr, rxThr_PIN);
}
void calc_Ail()  {
  calc_input(rxAil, rxAil_PIN);
}
void calc_Ele()  {
  calc_input(rxEle, rxEle_PIN);
}
void calc_Rud()  {
  calc_input(rxRud, rxRud_PIN);
}
void calc_Aux() {
  calc_input(rxAux, rxAux_PIN);
}


// *****     rx Servo Output     *****
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
    ltESCPulse = constrain(ltESCTmp, SERVO_ENDPOINT_LOW, SERVO_ENDPOINT_HIGH);
    rtESCTmp = rxPulse[rxThr] + upMod;
    rtESCPulse = constrain(rtESCTmp, SERVO_ENDPOINT_LOW, SERVO_ENDPOINT_HIGH);
  }
  else {  // Rudder Right
    ltESCTmp = reverse(rxPulse[rxThr] + upMod);
    ltESCPulse = constrain(ltESCTmp, SERVO_ENDPOINT_LOW, SERVO_ENDPOINT_HIGH);
    rtESCTmp = rxPulse[rxThr] - dwnMod;
    rtESCPulse = constrain(rtESCTmp, SERVO_ENDPOINT_LOW, SERVO_ENDPOINT_HIGH);
  }

#ifdef DEBUG_MIXING
  Serial.print("  MIXING   - rate: "); Serial.print(DTRate);
  Serial.print("  yawP: "); Serial.print(yaw_pct);
  Serial.print("  thrP: "); Serial.print(thr_pct);  
  Serial.print("  uMod: "); Serial.print(upMod);
  Serial.print("  dMod: "); Serial.print(dwnMod);
#endif  
}


// SETUP
void setup() {
  // General Setup
#ifdef DEBUG_SETUP
  Serial.begin(SERIAL_SPEED);
  Serial.println(" ");
  Serial.println("     Copyright (C) 2017 - Jim Lander (jamieFL)");  
  Serial.println("     This program comes with ABSOLUTELY NO WARRANTY.  It is free software: you can redistribute it and/or modify ");
  Serial.println("     it under the terms of the GNU General Public License.  For details, see http://www.gnu.org/licenses/");
  Serial.println(" ");
  unsigned long temp_time = timer2.get_count();
  while((timer2.get_count() - temp_time)/1000 < LOOP_DELAY);   //Wait until LOOP_DELAY passes.
#endif

  timer2.setup(); //this MUST be done before the other Timer2_Counter functions work; Note: since this messes up PWM outputs on pins 3 & 11, as well as 
                  //interferes with the tone() library (http://arduino.cc/en/reference/tone), you can always revert Timer2 back to normal by calling 
                  //timer2.unsetup()

  TIMSK0 &= ~_BV(TOIE0);    // disable timer0 overflow interrupt - NEEDED to reduce servo jitter

  //rx INPUTS
  pinMode(rxThr_PIN, INPUT);
  pinMode(rxAil_PIN, INPUT);
  pinMode(rxEle_PIN, INPUT);
  pinMode(rxRud_PIN, INPUT);
  pinMode(rxAux_PIN, INPUT);

  enableInterrupt(rxThr_PIN, calc_Thr, CHANGE);
  enableInterrupt(rxAil_PIN, calc_Ail, CHANGE);
  enableInterrupt(rxEle_PIN, calc_Ele, CHANGE);
  enableInterrupt(rxRud_PIN, calc_Rud, CHANGE);
  enableInterrupt(rxAux_PIN, calc_Aux, CHANGE);

  // Servo OUTPUT
  ServoArray[ltAil].attach(ltAil_PIN, 900, 2100); //  Left Elevon
  ServoArray[ltTV].attach (ltTV_PIN, 900, 2100);  //  Left Thrust Vector Servo
  ServoArray[ltESC].attach(ltESC_PIN, 900, 2100); //  Left ESC
  ServoArray[rtESC].attach(rtESC_PIN, 900, 2100); //  Right ESC
  ServoArray[rtTV].attach (rtTV_PIN, 900, 2100);  //  Right Thrust Vector Servo
  ServoArray[rtAil].attach(rtAil_PIN, 900, 2100); //  Right Elevon
}


// LOOP
void loop() {

  rxReadPulse();  // get current rx INPUTS

#ifdef DEBUG_RX
  Serial.print("  RECEIVER - Thr: "); Serial.print(rxPulse[rxThr]);  // Pring rx INPUTS
  Serial.print("  Ail:  "); Serial.print(rxPulse[rxAil]);
  Serial.print("  Ele:  "); Serial.print(rxPulse[rxEle]);
  Serial.print("  Rud:  "); Serial.print(rxPulse[rxRud]);
  Serial.print("  Aux: "); Serial.print(rxPulse[rxAux]);
#endif  

  // Perform Elevon, Thust Vectoring, and Differential Thrust Mixing
  if (rxPulse[rxAux] < 1250) {       // MODE 1
    mixElevon();                      // Elevon Mixing: ON
    rtTVPulse = 1500;                 // Thrust Vecoting: OFF
    ltTVPulse = 1500;
    rtESCPulse = rxPulse[rxThr];      // Differential Thrust: OFF
    ltESCPulse = reverse(rxPulse[rxThr]);
  }
  else if (rxPulse[rxAux] > 1750) {  // MODE 3
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

#ifdef DEBUG_SERVO
  Serial.print("  SERVOS   - ltAl: "); Serial.print(ltAilPulse);  // Print Servo OUTPUTS
  Serial.print("  rtAl: "); Serial.print(rtAilPulse);
  Serial.print("  ltTV: "); Serial.print(ltTVPulse);
  Serial.print("  rtTV: "); Serial.print(rtTVPulse);
  Serial.print("  lESC: "); Serial.print(ltESCPulse);
  Serial.print("  rESC: "); Serial.print(rtESCPulse);
#endif  

#ifdef DEBUG_SETUP
  Serial.println("");
  unsigned long temp_time = timer2.get_count();
  while((timer2.get_count() - temp_time)/1000 < LOOP_DELAY);   //Wait until LOOP_DELAY passes.
#endif

}
