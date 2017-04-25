# VECTOREDAIR

    This sketch is is designed for the Arduino Uno or Nano.  The Nano will 
    be installed between an FrSky S6r stabilized receiver, and a flying 
    wing's servos and ESCs. Its purpos is to perform Elevon, Differential 
    Thrust (DT), and Thrust Vectoring (TV) mixing.
    
# RECEIVER INPUTS
   Pin  8, PCINT0, Channel 1, Throttle</ br>
   Pin  9, PCINT1, Channel 2, Aileron</ br>
   Pin 10, PCINT2, Channel 3, Elevator</ br>
   Pin 11, PCINT3, Channel 4, Rudder</ br>
   Pin 12, PCINT4, Channel 5, Aux Channel</ br>
   
# SERVO & ESC OUTPUTS
   Pin 2, Left Aileron Servo</ br>
   Pin 3, Left Thrust Vectoring Servo</ br>
   Pin 4, Left Electronic Speed Controller (ESC)</ br>
   Pin 5, Right Electonic Speed Controller (ESC)</ br>
   Pin 6, Right Thrust Vectoring Servo</ br>
   PIN 7, Right Aileron Servo  </ br>
   
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
