
/*

 Derived from MultiWii GUI Telemetry.
 
 This is free software: you can redistribute it and/or modify
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


#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadTX,serialTailTX;
static uint8_t serialBufferTX[TX_BUFFER_SIZE];
static uint8_t inBuf[INBUF_SIZE+1];

#define PWM_OUTPUTS 4

int16_t pwm[PWM_OUTPUTS] = {
  0};

static struct { // not used by Adapter
  uint16_t thisVersion;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  uint16_t activate[CHECKBOX_ITEMS];
  uint16_t cycletimeuS;
  int16_t minthrottleuS;
  uint8_t checksum; // must be last! 
} 
conf;


static uint16_t cycleTimeuS = 2222;

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}

uint8_t read8()  {
  return inBuf[indRX++]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum);
  UartSendData();
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) 
    serialize8(pgm_read_byte(c));
}

void serialCom() {
  uint8_t c,n;  
  static uint8_t offset;
  static uint8_t dataSize;
  static enum _serial_state {
    WaitSentinel,
    WaitHeader1,
    WaitHeader2,
    WaitSize,
    WaitCmd,
    WaitPayload,
  } 
  MSPState;// = WaitSentinel;

  while (Serial.available() > 0) {
    c = Serial.read();
    // regular data handling to detect and handle MSP and other data
    if (MSPState == WaitSentinel)
      MSPState = (c=='$') ? WaitHeader1 : WaitSentinel;
    else if (MSPState == WaitHeader1) 
      MSPState = (c=='M') ? WaitHeader2 : WaitSentinel;
    else if (MSPState == WaitHeader2)
      MSPState = (c=='<') ? WaitSize : WaitSentinel; 
    else if (MSPState == WaitSize) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        MSPState = WaitSentinel;
        continue;
      }
      dataSize = c;
      offset = 0;
      checksum = 0;
      indRX = 0;
      checksum ^= c;
      MSPState = WaitCmd;  // the command is to follow
    } 
    else if (MSPState == WaitCmd) {
      cmdMSP = c;
      checksum ^= c;
      MSPState = WaitPayload;
    } 
    else if (MSPState == WaitPayload && offset < dataSize) {
      checksum ^= c;
      inBuf[offset++] = c;
    } 
    else if (MSPState == WaitPayload && offset >= dataSize) {
      if (checksum == c) 
        evaluateCommand();  
      MSPState = WaitSentinel;
    }
  }
} // serialCom


enum Sensors {
  SENS_ACC, SENS_BARO, SENS_MAG, SENS_GPS, SENS_SONAR, SENS_OPTIC};

void evaluateCommand() { 
  uint8_t len, i;
  int16_t Temp;

  switch(cmdMSP) {
  case MSP_SET_RAW_RC:
    for(i = 0; i < 8; i++) 
      rcData[i] = read16();
    headSerialReply(0);
    break;
  case MSP_SET_PID:
    for(i = 0; i < PIDITEMS; i++) {
      conf.P8[i]=read8();
      conf.I8[i]=read8();
      conf.D8[i]=read8();
    }
    headSerialReply(0);
    break;
  case MSP_SET_BOX:
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      conf.activate[i]=read16();
    headSerialReply(0);
    break;
  case MSP_SET_RC_TUNING:
    conf.rcRate8 = read8();
    conf.rcExpo8 = read8();
    conf.rollPitchRate = read8();
    conf.yawRate = read8();
    conf.dynThrPID = read8();
    conf.thrMid8 = read8();
    conf.thrExpo8 = read8();
    headSerialReply(0);
    break;
  case MSP_SET_MISC:
    headSerialReply(0);
    break;
  case MSP_IDENT:
    headSerialReply(7);
    serialize8(0);  
    serialize8(3); 
    serialize8(MSP_VERSION); 
    serialize32(0);//pgm_read_dword(&(capability))); // "capability"
    break;
  case MSP_STATUS:
    headSerialReply(11);
    serialize16(cycleTimeuS);
    serialize16(0);//i2cErrors);

    serialize16(0 ); 

    serialize32( !disableThrottle << BOX_ARM );
    serialize8(0); // current parameter set
    break;
  case MSP_RAW_IMU:
    headSerialReply(18);
    for(i = 0; i < 3; i++) serialize16(accData[i]);
    for(i = 0; i < 3; i++) serialize16(gyroData[i]);
    for(i = 0; i < 3; i++) serialize16(magADC[i]);
    break;
  case MSP_SERVO:
    headSerialReply(16);
    for(i = 0; i < 8; i++) {
      //  Temp = pwm[pwmMap[i]];
      //  if (pwmMap[i] != 0)
      //    Temp += MID_RC_US; 
      serialize16(0);//Temp);
    }
    break;
  case MSP_MOTOR:
    headSerialReply(16);
    for(i = 0; i < 8; i++)
      serialize16( (i < PWM_OUTPUTS) ? pwm[i] : 0 );
    break;
  case MSP_RC:
    headSerialReply(RC_CHANS * 2);
    for(i = 0; i < RC_CHANS; i++) 
      if (USING_HUBSAN) 
        serialize16((rcData[i] + 247) << 2);  
      else
        serialize16(rcData[i]);
    break;
  case MSP_ATTITUDE:
    headSerialReply(10);
    for(i = 0; i < 3; i++) serialize16(angle[i]);
    serialize16(0);
    serialize16(0);
    break;
  case MSP_ALTITUDE:
    headSerialReply(6);
    serialize32(0);//relativeAltitude);
    serialize16(0);//ROC);
    break;
  case MSP_ANALOG:
    headSerialReply(4);
    serialize8(batteryVolts);
    serialize16(0);//analog.intPowerMeterSum);
    serialize8(rssiBackChannel);
    break;
  case MSP_RC_TUNING:
    headSerialReply(7);
    serialize8(conf.rcRate8);
    serialize8(conf.rcExpo8);
    serialize8(conf.rollPitchRate);
    serialize8(conf.yawRate);
    serialize8(conf.dynThrPID);
    serialize8(conf.thrMid8);
    serialize8(conf.thrExpo8);
    break;
  case MSP_PID:
    headSerialReply(3*PIDITEMS);
    for(i = 0; i < PIDITEMS; i++) {
      serialize8(conf.P8[i]);
      serialize8(conf.I8[i]);
      serialize8(conf.D8[i]);
    }
    break;
  case MSP_PIDNAMES:
    headSerialReply(strlen_P(pidnames));
    serializeNames(pidnames);
    break;
  case MSP_BOX:
    headSerialReply(2*CHECKBOX_ITEMS);
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      serialize16(conf.activate[i]);
    break;
  case MSP_BOXNAMES:
    headSerialReply(strlen_P(boxnames));
    serializeNames(boxnames);
    break;
  case MSP_BOXIDS:
    headSerialReply(CHECKBOX_ITEMS);
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      serialize8(pgm_read_byte(&(boxids[i])));
    break;
  case MSP_MOTOR_PINS:
    headSerialReply(8);
    for(i = 0; i < 8; i++)
      serialize8(0);//PWM_PIN[i]);
    break;
  case MSP_RESET_CONF:
    // if(!f.ARMED) LoadDefaults();
    headSerialReply(0);
    break;
  case MSP_ACC_CALIBRATION:
    // if(!f.ARMED) calibratingA = 512;
    headSerialReply(0);
    break;
  case MSP_MAG_CALIBRATION:
    // if(!f.ARMED) f.CALIBRATE_MAG = true;
    headSerialReply(0);
    break;
  case MSP_EEPROM_WRITE:
    //writeParams(0);
    headSerialReply(0);
    break;
  case MSP_DEBUG:
    headSerialReply(8);
    for(i = 0; i < 4; i++)
      serialize16(debug[i]); // 4 variables are here for general monitoring purpose
    break;
  default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
    headSerialError(0);
    break;
  }
  tailSerialReply();
} // evaluatCommand


void serialize32(uint32_t a) {
  serialize8((a    ) & 0xff);
  serialize8((a>> 8) & 0xff);
  serialize8((a>>16) & 0xff);
  serialize8((a>>24) & 0xff);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xff);
  serialize8((a>>8) & 0xff);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX;

  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t] = a;
  checksum ^= a;
  serialHeadTX = t;
}

void UartSendData(void) {

  while(serialHeadTX != serialTailTX) {
    if (++serialTailTX >= TX_BUFFER_SIZE) 
      serialTailTX = 0;
    Serial.write(serialBufferTX[serialTailTX]);
  }

} // UartSendData

void SerialWrite(uint8_t port,uint8_t c){

  serialize8(c);
  UartSendData();
} // SerialWrite


































