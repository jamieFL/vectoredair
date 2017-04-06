# VECTOREDAIR

    This sketch is is designed for the Arduino Uno or Nano.  The Nano will 
    be installed between an FrSky S6r stabilized receiver, and a flying 
    wing's servos and ESCs. Its purpos is to perform Elevon, Differential 
    Thrust (DT), and Thrust Vectoring (TV) mixing.
    
# RECEIVER INPUTS
Pin 2, Channel 1, Throttle<BR />
Pin 3, Channel 2, Aileron<BR />
Pin 4, Channel 3, Elevator<BR />
Pin 5, Channel 4, Rudder<BR />
Pin 6, Channel 5, Auxilary Channle 1 - Mode Switch<BR />
Mode 1, PWM < 1250, TV Off, DT @ 15%<BR />
Mode 2, PWM ~1500, TV On, DT @ 15%<BR />
Mode 3, PWM > 1750, TV On, DT 100%<BR />
Pin 7, Channel 6, Auxilary Channel 2 - RESERVED<BR />
   
# SERVO & ESC OUTPUTS
Pin 8,  Left Aileron Servo<BR />
Pin 9,  Left Thrust Vectoring Servo<BR />
Pin 10, Left Electronic Speed Controller (ESC)<BR />
Pin 11, Right Electonic Speed Controller (ESC)<BR />
Pin 12, Right Thrust Vectoring Servo<BR />
PIN 13, Right Aileron Servo<BR />
   
# INCLUDED LIBRARIES
Enable Interrupt        http://tiny.cc/EnableInterrupt<BR />
eRCaGuy_Timer2_Counter  http://tiny.cc/eRCaGuy_Timer2_Counter<BR />
Servos                  http://tiny.cc/arduino-servo<BR />
   
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
