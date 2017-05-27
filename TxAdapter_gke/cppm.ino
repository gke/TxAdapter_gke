
/*
  Standalone Tx adapter version by Prof. G.K. Egan (gke) 2013.
  
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

// Rx

#define WidthOK(w) ((w>=900) && (w<=2200))

#define CPPM  THROTTLE,ROLL,PITCH,YAW,AUX1,AUX2,AUX3,AUX4 //For JR/Graupner/Spektrum

const uint8_t TxMap[] = {
  CPPM
};

static uint32_t rcTimeuS  = 0;
static boolean cppmFrameOK = false;

volatile uint16_t cppmRaw[RC_CHANS] = { 
  1502, }; // interval [1000;2000]

void cppmInit(void) {

  pinMode(3, INPUT);
  attachInterrupt(1, rxInt, RISING);

} // cppmInit

void rxInt(void) {
  uint32_t NowuS;
  int16_t Width;
  static uint32_t PrevuS = 0;
  static uint8_t chan = 0;

  NowuS = micros();
  interrupts();
  Width = NowuS - PrevuS;
  PrevuS = NowuS;
  if( Width > 3000) { 
    cppNewValues = cppmFrameOK;
    cppmFrameOK = true;
    chan = 0;
  }
  else
    if (chan < RC_CHANS) { 
      if (WidthOK(Width)) {
        cppmRaw[chan] = Width;
        LEDs(HIGH);
      } else {
        cppmFrameOK &= false;
         LEDs(LOW);
      }
      chan++;
    }

} // rxInt


void cppmGetInput(void) {

  // FlySky
  // -100% =~ 0x03e8 1000us (min)
  // +100% =~ 0x07ca 1994us (max)
  // Center = 0x5d9 1497us (center)
  
  // Hubsan
  // ...

  uint8_t chan;
  int16_t v;


  for (chan = 0; chan < RC_CHANS; chan++) {
    noInterrupts();
    v = cppmRaw[chan];
    interrupts();
   switch(currProtocol) {    
   case hubsan: rcData[TxMap[chan]] = constrain((v >> 2) - 247, 0, 255); break;
   case flysky: rcData[TxMap[chan]] = constrain(v - 1500 + 1497, 1000, 1994); break;
   default: break;
   } // switch;
  }

  cppmNewValues = false;

} // cppmGetInput



































