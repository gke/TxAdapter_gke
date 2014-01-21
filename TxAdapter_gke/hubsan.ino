
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



uint16_t hubsanVTXFreq;
uint8_t chan;
const uint8_t hubsanAllowedChannels[] = {
  0x14, 0x1e, 0x28, 0x32, 0x3c, 0x46, 0x50, 0x5a, 0x64, 0x6e, 0x78, 0x82};
uint32_t sessionID;
uint8_t hubsanPacketCount;
uint8_t hubsanTxState, hubsanRFMode;

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
  uint32_t timeoutuS;

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

  a7105WriteReg(A7105_02_CALC, 1);  
  vco_current = a7105ReadReg(A7105_02_CALC);

  timeoutuS = a7105SetTimeout();
  while (a7105Busy()) 
    if (micros() > timeoutuS) return (false);

  if (a7105ReadReg(A7105_22_IF_CALIB_I) & A7105_MASK_FBCF) return (false);

  a7105ReadReg(A7105_24_VCO_CURCAL);
  //a7105WriteReg(0x24, 0x13); // VCO cal. from A7105 Datasheet
  //a7105WriteReg(0x26, 0x3b); // VCO bank cal. limits from A7105 Datasheet

  a7105WriteReg(A7105_0F_CHANNEL, 0); // set channel
  a7105WriteReg(A7105_02_CALC, 2); // VCO cal.

  timeoutuS = a7105SetTimeout();
  while (a7105Busy()) 
    if (micros() > timeoutuS) return (false);

  if (a7105ReadReg(A7105_25_VCO_SBCAL_I) & A7105_MASK_VBCF) return (false);

  a7105WriteReg(A7105_0F_CHANNEL, 0xa0); // set channel         
  a7105WriteReg(A7105_02_CALC, 2); // VCO cal.

  timeoutuS = a7105SetTimeout();
  while (a7105Busy()) 
    if (micros() > timeoutuS) return (false);

  if (a7105ReadReg(A7105_25_VCO_SBCAL_I) & A7105_MASK_VBCF) return (false);

  //a7105WriteReg(0x25, 0x08); // reset VCO band cal.

  a7105SetPower(TXPOWER_150mW);
  a7105Strobe(A7105_STANDBY);

  return (true);

} // hubsanSetup


static void hubsanUpdateTelemetry(void) {
  enum Tag0xe0 {
    TAG, 
    ROC_MSB, // ?
    ROC_LSB,
    unknown_e0_3,
    unknown_e0_4, 
    unknown_e0_5,
    unknown_e0_6,
    unknown_e0_7,
    unknown_e0_8, 
    Z_ACC_MSB, 
    Z_ACC_LSB, 
    YAW_GYRO_MSB, 
    YAW_GYRO_LSB, 
    VBAT,
    CRC0,
    CRC1
  };

  enum Tag0xe1 {
    TAG_e1, 
    PITCH_ACC_MSB,
    PITCH_ACC_LSB,
    ROLL_ACC_MSB,
    ROLL_ACC_LSB, 
    unknown_e1_5,
    unknown_e1_6,
    PITCH_GYRO_MSB, 
    PITCH_GYRO_LSB, 
    ROLL_GYRO_MSB, 
    ROLL_GYRO_LSB, 
    unknown_e1_11,
    unknown_e1_12,  
    VBAT_e1,
    CRC0_e1,
    CRC1_e1
  };

  uint8_t i;
  static uint32_t lastUpdatemS = millis();
  uint16_t intervalmS;

  if ( a7105CRCCheck(16)) {
    intervalmS = millis() - lastUpdatemS;
    lastUpdatemS = millis();

    if (USING_MW_GUI)
      switch (packet[0]) {
      case 0xe0:
        estAltitude = -(packet[ROC_MSB] << 8 | packet[ROC_LSB]);// 1,2
        debug[0] = packet[3] << 8 | packet[4];
        debug[1] = packet[5] << 8 | packet[6]; 
        debug[2] = packet[7] << 8 | packet[8]; 
        accData[YAW] = packet[Z_ACC_MSB] << 8 | packet[Z_ACC_LSB]; // OK
        gyroData[YAW] = packet[YAW_GYRO_MSB] << 8 | packet[YAW_GYRO_LSB]; 
        batteryVolts = packet[VBAT];
        break;
      case 0xe1:
        accData[PITCH] = packet[PITCH_ACC_MSB] << 8 | packet[PITCH_ACC_LSB];  
        accData[ROLL] = packet[ROLL_ACC_MSB] << 8 | packet[ROLL_ACC_LSB]; 

        gyroData[PITCH] = packet[PITCH_GYRO_MSB] << 8 | packet[PITCH_GYRO_LSB]; 
        gyroData[ROLL] = packet[ROLL_GYRO_MSB] << 8 | packet[ROLL_GYRO_LSB]; 

        // use acc as angle alias
        angle[PITCH] = accData[PITCH];
        angle[ROLL] = -accData[ROLL];
        batteryVolts = packet[VBAT];
        break;
      default:
        break;
      }  
    else 
      if (DEBUG) {
      //  if (packet[0] == 0xe0) {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(packet[0], HEX);
      Serial.print(",");
      for (i = 1;i<12;i+=2) {
        Serial.print(packet[i] << 8 | packet[i+1]); 
        Serial.print(",");
      }
      Serial.print(packet[11]);  
      Serial.print(",");    
      Serial.print(packet[12]);
      Serial.print(",");
      Serial.print(packet[13]); 
      Serial.println();
      //   } 
    }
    else {    
      Serial.print(intervalmS);
      Serial.print(" ");
      Serial.print(rssi);  
      Serial.print(" ");
      Serial.print(rssiBackChannel);  
      Serial.print(" ");
      Serial.print(batteryVolts);
      Serial.print(" ");
      Serial.print(disableThrottle);
      Serial.print(" ");
      Serial.println(((uint32_t)throttleLVCScale * 100) >> 10);
    }
  }
} // hubsanUpdateTelemetry

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
  uint8_t currentThrottle;

  memset(packet, 0, 16);

#define VTX_STEP_SIZE 5

  currentThrottle = disableThrottle ? 0 : ((int32_t)rcData[THROTTLE] * throttleLVCScale) >> 10;

#if defined(USE_HUBSAN_EXTENDED)
  if ( hubsanPacketCount == 100 ) {// set vTX frequency (H107D)
    packet[0] = 0x40;
    packet[1] = (hubsanVTXFreq >> 8) & 0xff;
    packet[2] = hubsanVTXFreq & 0xff;
    packet[3] = 0x82;
    hubsanPacketCount++;      
  } 
  else
#endif
  { //20 00 00 00 80 00 7d 00 84 02 64 db 04 26 79 7b
    packet[0] = 0x20;
    packet[2] = currentThrottle;
  }
  packet[4] = 0xff - rcData[YAW];
  packet[6] = 0xff - rcData[PITCH];
  packet[8] = rcData[ROLL]; 

  packet[9] = 0x20;

debug[0] = rcData[AUX1];
debug[1] = rcData[AUX2];

  if (hubsanPacketCount > 100) {  // sends default value for the 100 first packets
    enableLEDs = rcData[AUX1] > 127;
    enableFlip = rcData[AUX2] > 127;
    if (enableLEDs)  bitSet(packet[9], HUBSAN_LEDS_BIT);
    if (enableFlip) bitSet(packet[9], HUBSAN_FLIP_BIT);
  }
  else {
    enableLEDs = enableFlip = true;
    hubsanPacketCount++;   
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
  enum {
    doTx, 
    waitTx, 
    pollRx
  };
  uint16_t d;
  static uint8_t telemetryState = doTx;
  static uint8_t polls, i;

  switch(hubsanState) {
    ledPeriodmS = BIND_LED_PERIOD_MS;
  case BIND_1:
  case BIND_3:
  case BIND_5:
  case BIND_7:
    hubsanBuildBindPacket(hubsanState == BIND_7 ? 9 : (hubsanState == BIND_5 ? 1 : hubsanState + 1 - BIND_1));
    a7105Strobe(A7105_STANDBY);
    a7105WriteData(packet, 16, chan);
    hubsanState |= WAIT_WRITE;
    d = 3000;
    break;
  case BIND_1 | WAIT_WRITE:
  case BIND_3 | WAIT_WRITE:
  case BIND_5 | WAIT_WRITE:
  case BIND_7 | WAIT_WRITE:
    if (a7105Busy()) // check for completion
      if (DEBUG_PROTOCOL) Serial.println("pwf");
    a7105Strobe(A7105_RX);
    hubsanState &= ~WAIT_WRITE;
    hubsanState++;
    d = 4500; // 7.5mS elapsed since last write
    break;
  case BIND_2:
  case BIND_4:
  case BIND_6:
    if (a7105Busy()) {
      if (DEBUG_PROTOCOL) Serial.println("ns");
      hubsanState = BIND_1; 
      d = 4500; // No signal, restart binding procedure.  12mS elapsed since last write
    } 
    else {
      a7105ReadData(packet, 16);
      hubsanState++;
      if (hubsanState == BIND_5)
        a7105WriteID((packet[2] << 24) | (packet[3] << 16) | (packet[4] << 8) | packet[5]);
      d = 500;  // 8mS elapsed time since last write
    }
    break;
  case BIND_8:
    if (a7105Busy()) {
      if (DEBUG_PROTOCOL) Serial.println("nr");
      hubsanState = BIND_7;
      d = 15000; // 22.5mS elapsed since last write
    } 
    else {
      if (DEBUG_PROTOCOL) Serial.println("r");
      a7105ReadData(packet, 16);
      if (packet[1] == 9) {
        hubsanState = DATA_1;
        a7105WriteReg(A7105_1F_CODE_I, 0x0F);
        hubsanSetBindState(0);
        d = 28000; // 35.5mS elapsed since last write
      } 
      else {
        hubsanState = BIND_7;
        d = 15000; // 22.5 mS elapsed since last write
      }
    }
    break;
  case DATA_1:
    ledPeriodmS = HUBSAN_LED_PERIOD_MS;
    a7105SetPower(TXPOWER_150mW); // keep transmit power in sync
  case DATA_2:
  case DATA_3:
  case DATA_4:
  case DATA_5:
    switch (telemetryState) { // Goebish - telemetry is every ~0.1S r 10 Tx packets
    case doTx:
      hubsanBuildPacket();
      a7105Strobe(A7105_STANDBY);
      a7105WriteData(packet, 16, hubsanState == DATA_5 ? chan + 0x23 : chan);
      d = 3000; // nominal tx time
      telemetryState = waitTx;
      break;
    case waitTx: 
      if (a7105Busy())
        d = 0;
      else { // wait for tx completion
        Probe();
        a7105Strobe(A7105_RX);
        polls = 0; 
        telemetryState = pollRx;
        d = 3000; // nominal rx time
      } 
      break;
    case pollRx: // check for telemetry
      if (a7105Busy()) 
        d = 1000;
      else { 
        Probe();
        a7105ReadData(packet, 16);
        rssiBackChannel = a7105ReadReg(A7105_1D_RSSI_THOLD);
        hubsanUpdateTelemetry();
        a7105Strobe(A7105_RX);
        d = 1000; 
      }

      if (++polls >= 7) { // 3ms + 3mS + 4*1ms
        if (hubsanState == DATA_5) 
          hubsanState = DATA_1;
        else 
          hubsanState++;  
        telemetryState = doTx;   
      }
      break;
    } // switch
    break;
  } // switch

  return (d);
} // hubsanUpdate

static void hubsanInit(void) {

  ledPeriodmS = BIND_LED_PERIOD_MS;
  while (!hubsanSetup()) {
    if (DEBUG_PROTOCOL)
      Serial.println("hubsanSetup FAIL");
    checkLEDFlash(); 
  }

  if (DEBUG_PROTOCOL) Serial.println("hubsanSetup OK");

  sessionID = random();
  chan = hubsanAllowedChannels[random() % sizeof(hubsanAllowedChannels)];
  hubsanSetBindState(0xffffffff);
  hubsanState = BIND_1;
  hubsanPacketCount = 0;
  hubsanVTXFreq = DEFAULT_VTX_FREQ;

} // hubsanInit



























