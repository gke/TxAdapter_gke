
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
 Midelic.
 
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

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "a7105.h"

static boolean cppmNewValues = false;
int16_t hubsanState;

uint8_t checkID[4];

uint8_t packet[21];
int16_t rcData[RC_CHANS] = {
  0,};

uint8_t batteryVolts = 255;
uint8_t rssiBackChannel = 0;
int16_t throttleLVCScale = 1024;
boolean disableThrottle = false;

int16_t accData[3] = {
  0};
int16_t gyroData[3] = {
  0};
int16_t magADC[3] = {
  0};
int16_t angle[3] = {
  0};
int16_t debug[4] = {
  0};

void Probe(void) {
  digitalWrite(A0, LOW);
  digitalWrite(A0, HIGH);
  digitalWrite(A0, LOW);
} // Probe

void LEDs(boolean s) {
  digitalWrite(LED_PIN, s); 
  digitalWrite(LED2_PIN, s); 
} // LEDs

void Alarm(boolean active) {
  static uint8_t ledState = 0;

  if (active) {
    ledState = digitalRead(LED_PIN);
    ledToggle(200);
  }
  else LEDs(ledState); 

} // Alarm

void ledToggle(uint16_t period) {
  static uint32_t UpdatemS = millis();

  if (millis() > UpdatemS) {
    if (digitalRead(LED_PIN)) LEDs(false);
    else LEDs(true);
    UpdatemS += period;
  }
} // ledToggle

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(LED_PIN, OUTPUT); 
  pinMode(LED2_PIN, OUTPUT);

  pinMode(A0, OUTPUT);
  pinMode(PROTOCOL_PIN, INPUT);

 // causing problems with rebinding 
 // randomSeed((uint32_t)analogRead(A1) << 10 | analogRead(A2));  

  a7105Setup();

  if (USING_HUBSAN)
    hubsanInit();
  else
    flyskyInit();

  cppmInit();

} // setup

void loop() {
  static uint32_t UpdateuS = micros();
  uint32_t NowuS;
  uint8_t chan;

  if (USING_HUBSAN) {
    checkBattery();
    if (USING_MW_GUI)
      serialCom();
  }

  if (cppmNewValues) 
    cppmGetInput();

  NowuS = micros();
  if ( NowuS > UpdateuS )    
    UpdateuS = (USING_HUBSAN) ? NowuS +  hubsanUpdate() : NowuS +  flyskyUpdate();

} // loop


































