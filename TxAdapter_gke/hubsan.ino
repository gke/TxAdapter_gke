
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

const uint32_t hubsanID = 0xdb042679; // choose your own random value

const uint16_t DEFAULT_VTX_FREQ = 5885;

uint16_t hubsanVTXFreq;
uint8_t chan;
const uint8_t hubsanAllowedChannels[] = {
  0x14, 0x1e, 0x28, 0x32, 0x3c, 0x46, 0x50, 0x5a, 0x64, 0x6e, 0x78, 0x82};
uint32_t sessionID;
uint8_t hubsanPacketCount;

enum {
  BIND_1,
  BIND_2,
  BIND_3,
  BIND_4,
  BIND_5,
  BIND_6,
  BIND_7,
  BIND_8,
  DATA_1,
  DATA_2,
  DATA_3,
  DATA_4,
  DATA_5,
};

#define WAIT_WRITE 0x80

static uint32_t bind_time;

#define READY   0x01
#define BINDING 0x02
#define BINDDLG 0x04

void hubsanSetBindState(uint32_t ms) {
  if (ms) {
    if (ms == 0xFFFFFFFF)
      bind_time = ms;
    else
      bind_time = millis() + ms;
    hubsanState |= BINDING;
  } 
  else 
    hubsanState &= ~BINDING;
} // hubsanSetBindState

boolean hubsanSetup(void) {
  uint8_t vco_current;

  a7105Reset();

  a7105WriteID(0x55201041);
  a7105WriteReg(A7105_01_MODE_CONTROL, 0x63);
  a7105WriteReg(A7105_03_FIFOI, 0x0f);
  a7105WriteReg(A7105_0D_CLOCK, 0x05);
  a7105WriteReg(A7105_0E_DATA_RATE, 0x04);
  a7105WriteReg(A7105_15_TX_II, 0x2b);
  a7105WriteReg(A7105_18_RX, 0x62);
  a7105WriteReg(A7105_19_RX_GAIN_I, 0x80);
  a7105WriteReg(A7105_1C_RX_GAIN_IV, 0x0A);
  a7105WriteReg(A7105_1F_CODE_I, 0x07);
  a7105WriteReg(A7105_20_CODE_II, 0x17);
  a7105WriteReg(A7105_29_RX_DEM_TEST_I, 0x47);

  a7105Strobe(A7105_STANDBY);

  a7105WriteReg(A7105_02_CALC, 1); //IF Filter Bank Calibration  
  vco_current = a7105ReadReg(A7105_02_CALC);

  if (a7105FlagDone(A7105_02_CALC)) {

    if (!( a7105ReadReg(A7105_22_IF_CALIB_I) & A7105_MASK_FBCF)) {

      a7105ReadReg(A7105_24_VCO_CURCAL);

      //a7105WriteReg(0x24, 0x13); //Recomended VCO calibration from A7105 Datasheet
      //a7105WriteReg(0x26, 0x3b); //VCO Bank Calibration recomended limits from A7105 Datasheet

      a7105WriteReg(A7105_0F_CHANNEL, 0); //Set Channel
      a7105WriteReg(A7105_02_CALC, 2);  //VCO Calibration

      if (a7105FlagDone(A7105_02_CALC)) {

        if (!(a7105ReadReg(A7105_25_VCO_SBCAL_I) & A7105_MASK_VBCF)) {

          a7105WriteReg(A7105_0F_CHANNEL, 0xa0); //Set Channel         
          a7105WriteReg(A7105_02_CALC, 2); //VCO Calibration

          if (a7105FlagDone(A7105_02_CALC)) {

            if (!(a7105ReadReg(A7105_25_VCO_SBCAL_I) & A7105_MASK_VBCF)) { 

              //a7105WriteReg(0x25, 0x08); //Reset VCO Band calibration

              a7105SetPower(TXPOWER_150mW);
              a7105Strobe(A7105_STANDBY);

              return (true);
            }    
          } 
        }
      } 
    } 
  }
  return (false);
} // hubsanSetup

static void hubsanBuildBindPacket(uint8_t hubsanState) {

  packet[0] = hubsanState;
  packet[1] = chan;
  packet[2] = (sessionID >> 24) & 0xff;
  packet[3] = (sessionID >> 16) & 0xff;
  packet[4] = (sessionID >>  8) & 0xff;
  packet[5] = (sessionID >>  0) & 0xff;
  packet[6] = 0x08;
  packet[7] = 0xe4; //???
  packet[8] = 0xea;
  packet[9] = 0x9e;
  packet[10] = 0x50;
  packet[11] = (hubsanID >> 24) & 0xff;
  packet[12] = (hubsanID >> 16) & 0xff;
  packet[13] = (hubsanID >>  8) & 0xff;
  packet[14] = (hubsanID >>  0) & 0xff;

  a7105CRCUpdate(16);

} // hubsanBuildBindPacket

static void hubsanBuildPacket(void) {

  memset(packet, 0, 16);

#define VTX_STEP_SIZE "5"
  enum{
    FLAG_FLIP = 0x08,
    FLAG_LED  = 0x04
  };

//#define USE_HUBSAN_H107D

#if defined(USE_HUBSAN_H107D)
  if(hubsanVTXFreq != DEFAULT_VTX_FREQ || hubsanPacketCount == 100) {// set vTX frequency (H107D)
    hubsanVTXFreq = DEFAULT_VTX_FREQ;
    packet[0] = 0x40;
    packet[1] = (hubsanVTXFreq >> 8) & 0xff;
    packet[2] = hubsanVTXFreq & 0xff;
    packet[3] = 0x82;
    hubsanPacketCount++;      
  } 
  else { //20 00 00 00 80 00 7d 00 84 02 64 db 04 26 79 7b
    packet[0] = 0x20;
    packet[2] = rcData[THROTTLE];
  }
#else
  //20 00 00 00 80 00 7d 00 84 02 64 db 04 26 79 7b
  packet[0] = 0x20;
  packet[2] = rcData[THROTTLE];
#endif
  packet[4] = 0xff - rcData[YAW];
  packet[6] = 0xff - rcData[PITCH];
  packet[8] = rcData[ROLL]; 
  if (hubsanPacketCount < 100) {
    packet[9] = 0x02 | FLAG_LED | FLAG_FLIP; // sends default value for the 100 first packets
    hubsanPacketCount++;
  } else {
    packet[9] = 0x02;
    if(rcData[AUX1] >= 0) packet[9] |= FLAG_LED;
    if(rcData[AUX2] >= 0) packet[9] |= FLAG_FLIP;
  }

  packet[10] = 0x64;
  // could be optimised out
  packet[11] = (hubsanID >> 24) & 0xff;
  packet[12] = (hubsanID >> 16) & 0xff;
  packet[13] = (hubsanID >>  8) & 0xff;
  packet[14] = (hubsanID >>  0) & 0xff;

  a7105CRCUpdate(16);

} // hubsanBuildPacket

static uint16_t hubsanUpdate(void) {
  uint8_t i;

  switch(hubsanState) {
  case BIND_1:
  case BIND_3:
  case BIND_5:
  case BIND_7:
    if (DEBUG) Serial.println("b"); 
    hubsanBuildBindPacket(hubsanState == BIND_7 ? 9 : (hubsanState == BIND_5 ? 1 : hubsanState + 1 - BIND_1));
    a7105Strobe(A7105_STANDBY);
    a7105WriteData(packet, 16, chan);
    hubsanState |= WAIT_WRITE;
    return 3000;
  case BIND_1 | WAIT_WRITE:
  case BIND_3 | WAIT_WRITE:
  case BIND_5 | WAIT_WRITE:
  case BIND_7 | WAIT_WRITE:
    //wait for completion
    for(i = 0; i < 20; i++)
      if (!(a7105ReadReg(A7105_00_MODE) & 0x01))
        break;

    if (i >= 20)
      Serial.println("Failed to complete write\n");
    // else 
    //     Serial.println("Completed write\n");
    a7105Strobe(A7105_RX);
    hubsanState &= ~WAIT_WRITE;
    hubsanState++;
    return 4500; //7.5msec elapsed since last write
  case BIND_2:
  case BIND_4:
  case BIND_6:
    if (a7105ReadReg(A7105_00_MODE) & 0x01) {
      hubsanState = BIND_1; //Serial.println("Restart");
      return 4500; //No signal, restart binding procedure.  12msec elapsed since last write
    } 

    a7105ReadData(packet, 16);
    hubsanState++;
    if (hubsanState == BIND_5)
      a7105WriteID((packet[2] << 24) | (packet[3] << 16) | (packet[4] << 8) | packet[5]);

    return 500;  // 8msec elapsed time since last write;
  case BIND_8:
    if (a7105ReadReg(A7105_00_MODE) & 0x01) {
      hubsanState = BIND_7;
      return 15000; //22.5msec elapsed since last write
    }
    if (DEBUG) Serial.println("r");
    a7105ReadData(packet, 16);
    if (packet[1] == 9) {
      hubsanState = DATA_1;
      a7105WriteReg(A7105_1F_CODE_I, 0x0F);
      hubsanSetBindState(0);
      return 28000; //35.5msec elapsed since last write
    } 
    else {
      hubsanState = BIND_7;
      return 15000; //22.5 msec elapsed since last write
    }
  case DATA_1:
    //Keep transmit power in sync
    a7105SetPower(TXPOWER_150mW);
  case DATA_2:
  case DATA_3:
  case DATA_4:
  case DATA_5:
    if (DEBUG) Serial.println("w");
    hubsanBuildPacket();
    a7105WriteData(packet, 16, hubsanState == DATA_5 ? chan + 0x23 : chan);
    if (hubsanState == DATA_5)
      hubsanState = DATA_1;
    else
      hubsanState++;
    return 10000;
  }
  return 0;
} // hubsanUpdate

static void hubsanInit(void) {

  while (!hubsanSetup());

  sessionID = random();
  chan = hubsanAllowedChannels[rand() % sizeof(hubsanAllowedChannels)];
  hubsanSetBindState(0xffffffff);
  hubsanState = BIND_1;
  hubsanPacketCount = 0;
  hubsanVTXFreq = 0;

} // hubsanInit













