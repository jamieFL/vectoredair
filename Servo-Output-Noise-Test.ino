/**************************************************************************
    This sketch eliminates the read from RC receiver, simulates mixing
    calculations and outputs a stable servo signal to test for electrical
    issues that may be causing servo jitter.  If no jitter is present when
    this sketch is running, then dirty electical signal can be eliminate,
    and the output side of the sketch can also be deemed reliable.

   SERVO & ESC OUTPUTS
   Pin 8,  Left Aileron Servo
   Pin 9,  Left Thrust Vectoring Servo
   Pin 10, Left Electronic Speed Controller (ESC)
   Pin 11, Right Electonic Speed Controller (ESC)
   Pin 12, Right Thrust Vectoring Servo
   PIN 13, Right Aileron Servo

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

// #include <EnableInterrupt.h>
#include <Servo.h>
#include <eRCaGuy_Timer2_Counter.h>

#define DEBUG
#define SERIAL_SPEED 115200
#define DELAY 2000000                // If Servos attached, use small value

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

// *****     rx INPUT     *****
uint16_t rxPulse[RX_TOTAL_CHANNELS];

// *****     rx Servo Output     *****
volatile uint32_t ltAilPulse, ltTVPulse, ltESCPulse, rtESCPulse, rtTVPulse, rtAilPulse;
volatile float DTRate;
volatile unsigned long previousMillis;
volatile uint32_t loopPulse, tempLoopPulse;

Servo ServoArray[SERVO_TOTAL_CHANNELS];

// *****     rx INPUTS     *****
void rxReadPulse() {    
    tempLoopPulse = loopPulse + 250;
    if (tempLoopPulse > 2000) { loopPulse = 1000; }
    else { loopPulse = tempLoopPulse; }
      
    for (int i=0; i < RX_TOTAL_CHANNELS; i++){ 
      rxPulse[i]=loopPulse;
    }
    rxPulse[rxRud] = 1500;
    rxPulse[rxAil] = 1500;
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


// SETUP
void setup() {
  
  // General Setup
  timer2.setup(); //this MUST be done before the other Timer2_Counter functions work; Note: since this messes up PWM outputs on pins 3 & 11, as well as 
                  //interferes with the tone() library (http://arduino.cc/en/reference/tone), you can always revert Timer2 back to normal by calling 
                  //timer2.unsetup()

  TIMSK0 &= ~_BV(TOIE0);    // disable timer0 overflow interrupt - NEEDED to reduce servo jitter

  previousMillis = timer2.get_count()/2;
  loopPulse = 1000;
  tempLoopPulse = 1000;

#ifdef DEBUG
  Serial.begin(SERIAL_SPEED);
  Serial.println(" ");
  Serial.println("     Copyright (C) 2017 - Jim Lander (jamieFL)");  
  Serial.println("     This program comes with ABSOLUTELY NO WARRANTY.  It is free software: you can redistribute it and/or modify ");
  Serial.println("     it under the terms of the GNU General Public License.  For details, see http://www.gnu.org/licenses/");
  Serial.println(" ");
#endif

  //rx INPUTS Setup
  for (int i=0; i < RX_TOTAL_CHANNELS; i++){      // Set initial input values to neutral
    rxPulse[i]=1500;
  }

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
  unsigned long currentMillis;

  currentMillis = timer2.get_count()/2;
  if(currentMillis - previousMillis > DELAY) {
    previousMillis = currentMillis;

    rxReadPulse();

    #ifdef DEBUG
      Serial.print("  RECEIVER - Thr: "); Serial.print(rxPulse[rxThr]);  // Pring rx INPUTS
      Serial.print("  Ail:  "); Serial.print(rxPulse[rxAil]);
      Serial.print("  Ele:  "); Serial.print(rxPulse[rxEle]);
      Serial.print("  Rud:  "); Serial.print(rxPulse[rxRud]);
      Serial.print("  Aux: "); Serial.print(rxPulse[rxAux]);
    #endif 
    
    mixElevon();                      // Elevon Mixing: ON
    ltTVPulse = 1500;
    rtTVPulse = 1500;
    DTRate = dtHighRate;              // Differential Thrust: ON at HIGHT Rate
    mixThrottle();

    ServoArray[ltAil].writeMicroseconds(ltAilPulse);
    ServoArray[ltTV].writeMicroseconds(ltTVPulse);
    ServoArray[ltESC].writeMicroseconds(ltESCPulse);
    ServoArray[rtESC].writeMicroseconds(rtESCPulse);
    ServoArray[rtTV].writeMicroseconds(rtTVPulse);
    ServoArray[rtAil].writeMicroseconds(rtAilPulse);

    #ifdef DEBUG
      Serial.print("  SERVOS   - ltAl: "); Serial.print(ltAilPulse);  // Print Servo OUTPUTS
      Serial.print("  rtAl: "); Serial.print(rtAilPulse);
      Serial.print("  ltTV: "); Serial.print(ltTVPulse);
      Serial.print("  rtTV: "); Serial.print(rtTVPulse);
      Serial.print("  lESC: "); Serial.print(ltESCPulse);
      Serial.print("  rESC: "); Serial.println(rtESCPulse);
    #endif  
    
  }

}
