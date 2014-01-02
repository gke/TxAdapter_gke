
/*
  Standalone Tx adapter version by Prof. G.K. Egan (gke) 2013.
 
  Almost all of the adapter code was originally derived from the work 
  of PhracturedBlue and others for the universal transmitter "Deviation". 
  
  There have been numerous modifications, and in some cases rewrites, 
  of the original code.
  
  https://bitbucket.org/PhracturedBlue/deviation

  The adapter requires an a7105 transceiver and an Arduino Pro Mini or similar 
  preferably at 3.3V.  It is possible to modify the original Turnigy 9X 
  removable Tx module using its a7105 transciever.

  Pinouts are defined in config.h and the defaults are those adopted by 
  Midelic whose source code is not open.
  
  http://www.rcgroups.com/forums/showthread.php?t=1954078&highlight=hubsan+adapt
 
  This project is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  Deviation is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License 
  if not, see <http://www.gnu.org/licenses/>.
 
*/

//#define USE_TEST_DATA

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "a7105.h"

const boolean DEBUG = false;

static boolean cppmNewValues = false;
int16_t hubsanState;

uint8_t checkID[4];

uint8_t packet[21];
int16_t rcData[RC_CHANS] = {
  0,};

void ledToggle(uint16_t period) {
  static uint32_t UpdatemS = millis();

  if (millis() > UpdatemS) {
    if (digitalRead(LED_PIN)) 
      digitalWrite(LED_PIN, LOW); 
    else 
      digitalWrite(LED_PIN, HIGH);
    UpdatemS += period;
  }
} // ledToggle

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); 
  
  a7105Setup();
  
  if (USING_HUBSAN)
    hubsanInit();
  else
    flyskyInit();

//#if !defined(USE_TEST_DATA)
  cppmInit();
//#endif
} // setup

void loop() {
  static uint32_t UpdateuS = micros();
  uint8_t chan;

  if (cppmNewValues) 
    cppmGetInput();

  if ( micros() > UpdateuS )    
    if (USING_HUBSAN)
      UpdateuS = micros() +  hubsanUpdate(); 
    else
      UpdateuS = micros() +  flyskyUpdate();

} // loop






















