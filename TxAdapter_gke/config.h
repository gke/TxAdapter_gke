
//_______________________________________________________________________________________________

// User settings

#define hubsanID 0x30503136 // choose your own random IDs
#define flyskyID 0x35003136 

// Arduino pin assignments - uses Midelic defaults
#define CS_PIN 2 
#define PPM_PIN 3 
#define SCLK_PIN 4
#define SDIO_PIN 5
#define PROTOCOL_PIN 6 // for protocol selection switch
#define LED2_PIN A3 // binding LED - standard Arduino LED also flashes

#define USING_HUBSAN true // digitalRead(PROTOCOL_PIN)

//#define USE_HUBSAN_H107D
#define DEFAULT_VTX_FREQ 5885 // 0.001GHz

#define LVC_LIMIT 32 // 0.1V - LED flashes below this voltage for Hubsan

#define DEBUG false
#define DEBUG_PROTOCOL false
#define USING_MW_GUI (!(DEBUG || DEBUG_PROTOCOL)) 
#define SERIAL_BAUD_RATE 57600 // unfortunately 115200 does not work for 8MHz AtMega328

//_______________________________________________________________________________________________

// Should not need changing

#define Limit1(i,l) (((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))
#define Sign(n) ((n<0) ? -1 : 1)

#define LED_PIN 13 // Arduino SCK pin LED - don't change

#define RC_CHANS 8

enum rc { // must be in this order
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum TxPower {
    TXPOWER_100uW,
    TXPOWER_300uW,
    TXPOWER_1mW,
    TXPOWER_3mW,
    TXPOWER_10mW,
    TXPOWER_30mW,
    TXPOWER_100mW,
    TXPOWER_150mW,
    TXPOWER_LAST,
};


enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL, 
  PIDITEMS
};

enum box {
  BOX_ARM,
  CHECKBOX_ITEMS
};


const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
"ARM;"
;

const char pidnames[] PROGMEM =
"ROLL;"
"PITCH;"
"YAW;"
"ALT;"
"Pos;"
"PosR;"
"NavR;"
"LEVEL;"
"MAG;"
"VEL;"
;

const uint8_t boxids[] PROGMEM = {
  1 << BOX_ARM
};

