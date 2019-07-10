/*
   BindSpektrum.ino : Standalone sketch to bind a Spektrum satellite receiver
   (e.g., Lemon RX) using a 5-volt Arduino (e.g., Uno, Mega)

   Adapted from code in http://www.makedronesbook.com/sites/makedronesbook.com/files/spekbind.ino_.zip

   This file is part of DSMRX.

   DSMRX is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   DSMRX is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with DSMRX.  If not, see <http://www.gnu.org/licenses/>.
 */

// pins
static const uint8_t SPEK_BIND_DATA   = 0;
static const uint8_t SPEK_BIND_GROUND = 4;
static const uint8_t SPEK_BIND_POWER  = 5;

// For DSM2
static const uint8_t SPEK_BIND_PULSES = 3;

// For DSMX
//static const uint8_t SPEK_BIND_PULSES = 9;


void setup() 
{
    pinMode(SPEK_BIND_DATA, INPUT);     // Data line from sat
    digitalWrite(SPEK_BIND_DATA,LOW);   // Turn off internal Pull Up resistor

    pinMode(SPEK_BIND_GROUND, INPUT);
    digitalWrite(SPEK_BIND_GROUND,LOW);
    pinMode(SPEK_BIND_GROUND, OUTPUT);
    digitalWrite(SPEK_BIND_GROUND,LOW);

    pinMode(SPEK_BIND_POWER, INPUT);
    digitalWrite(SPEK_BIND_POWER,LOW);
    pinMode(SPEK_BIND_POWER,OUTPUT);

    while(true) {  //Do not return.  User presses reset button to return to normal. 

        digitalWrite(SPEK_BIND_POWER,LOW); // Power off sat
        pinMode(SPEK_BIND_DATA, OUTPUT); 
        digitalWrite(SPEK_BIND_DATA,LOW); 
        delay(1000); 

        digitalWrite(SPEK_BIND_POWER,HIGH); // Power on sat
        delay(10);
        digitalWrite(SPEK_BIND_DATA,HIGH); 
        delay(60);                 // Keep data pin steady for 20 to 120ms after power up

        noInterrupts();    
        for (byte i = 0; i < SPEK_BIND_PULSES; i++) { 
            digitalWrite(SPEK_BIND_DATA,LOW); 
            delayMicroseconds(118);
            digitalWrite(SPEK_BIND_DATA,HIGH);
            delayMicroseconds(122);
        }
        interrupts();
        delay(60000);         //Allow one full minute to bind, then try again.
    }
}


void loop() 
{

}
