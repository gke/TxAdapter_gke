
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
#include <EEPROM.h>
#include "config.h"
#include "a7105.h"

enum {
  hubsan, flysky, etc};
uint8_t currProtocol;

static boolean cppmNewValues = false;
int16_t hubsanState;

uint8_t checkID[4];

uint8_t packet[21];
int16_t rcData[RC_CHANS] = {
  0,};

uint8_t batteryVolts = 255;
uint8_t rssiBackChannel = 0;
int16_t rssi = 0;
int16_t throttleLVCScale = 1024;
boolean disableThrottle = false;
boolean alarmBattery = false;
boolean alarmRSSI = false;

uint16_t ledPeriodmS = 5000;

int16_t estAltitude;
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
  digitalWrite(PROBE_PIN, LOW);
  digitalWrite(PROBE_PIN, HIGH);
  digitalWrite(PROBE_PIN, LOW);
} // Probe

inline void LEDs(boolean s) {
  digitalWrite(LED_PIN, s); 
  digitalWrite(LED2_PIN, s); 
} // LEDs

void checkAlarm(void) {
  enum { 
    buzzerWait, buzzerOn, buzzerOff                                   };
  static uint8_t buzzerState = buzzerWait;
  static uint32_t UpdatemS;
  boolean alarmActive;

  alarmActive = alarmBattery || alarmRSSI; // add other alarm sources as desired

  switch(buzzerState) {
  case buzzerWait: 
    if (alarmActive) { 
      digitalWrite(BUZZER_PIN, LOW);
      UpdatemS = millis() + BUZZER_ON_TIME_MS;
      buzzerState = buzzerOn;
    }
    break;
  case buzzerOn: 
    if (!alarmActive) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerState = buzzerWait;
    }
    else
      if (millis() > UpdatemS) {
        digitalWrite(BUZZER_PIN, HIGH);
        UpdatemS = millis() + BUZZER_OFF_TIME_MS; 
        buzzerState = buzzerOff;   
      }
    break;
  case buzzerOff:
    if ((millis() > UpdatemS) || !alarmActive) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerState = buzzerWait;
    }
    break;
  }
} // Alarm

void checkLEDFlash(void) {
  static uint32_t UpdatemS = millis() + ledPeriodmS;

  if (millis() > UpdatemS) {
    if (digitalRead(LED_PIN)) LEDs(false);
    else LEDs(true);
    UpdatemS += ledPeriodmS;
  }
} // checkLEDFlash

void initRF(void) {

  a7105Setup();
  switch (currProtocol) {
  case hubsan:
    hubsanInit(); 
    break;
  case flysky: 
    flyskyInit(); 
    break;
  } // switch

} // initRF

void setProtocol(void) {

  currProtocol = EEPROM.read(7);

  if(!digitalRead(PROTOCOL_PIN)) {
    switch (currProtocol ) {
    case flysky:
      currProtocol = hubsan;
      ledPeriodmS = HUBSAN_LED_PERIOD_MS;
      break;
    default:
    case hubsan:
      currProtocol = flysky;
      ledPeriodmS = FLYSKY_LED_PERIOD_MS;
      break;
    } // switch
    EEPROM.write(7, currProtocol);
  } 

  if (DEBUG_PROTOCOL) {
    Serial.print("Protocol ");
    Serial.println(currProtocol);
  }

} // setProtocol

void setup() {
  uint8_t p;

  Serial.begin(57600);//SERIAL_BAUD_RATE);

  pinMode(LED_PIN, OUTPUT); 
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  LEDs(HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  pinMode(PROBE_PIN, OUTPUT);
  pinMode(PROTOCOL_PIN, INPUT_PULLUP);

  // causing problems with rebinding 
  // randomSeed((uint32_t)analogRead(A1) << 10 | analogRead(A2));  

  setProtocol();

  cppmInit();
  LEDs(LOW);
  while (!cppmNewValues) // get at least one good PPM frame
    cppmGetInput();
  LEDs(HIGH);

  initRF(); 

} // setup


void loop() {
  static uint32_t UpdateuS = micros();
  uint32_t NowuS;
  uint8_t chan;

  if (currProtocol == hubsan) {
    checkBattery();
    if (USING_MW_GUI)
      serialCom();
  }

  if (cppmNewValues) 
    cppmGetInput();

  NowuS = micros();
  if ( NowuS > UpdateuS )  
    UpdateuS = (currProtocol == hubsan) ? NowuS +  hubsanUpdate() : NowuS +  flyskyUpdate();

  checkAlarm();
  checkLEDFlash();

} // loop























































