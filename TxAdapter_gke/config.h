
//_______________________________________________________________________________________________

// User settings

#define hubsanID 0x35000001 // choose your own random IDs
#define flyskyID 0x5475c52a

// Arduino pin assignments - uses Midelic defaults
#define CS_PIN 2 
#define PPM_PIN 3 
#define SCLK_PIN 4
#define SDIO_PIN 5
#define PROTOCOL_PIN 6 // for protocol selection switch
#define BUZZER_PIN 6 // for protocol selection switch
#define LED2_PIN A3 // binding LED - standard Arduino LED also flashes
#define VBAT_PIN A0 // for Tx adapters with battery

#define PROBE_PIN A1 // for dubbing timing etc.

#define USE_HUBSAN_EXTENDED true // H107D LED/Flip etc control on AUX channels
#define DEFAULT_VTX_FREQ 5885 // x0.001GHz

#define USE_WLTOYS_EXTENDED false // LED/Flip etc control on AUX channels

#define LVC_LIMIT 32 // 0.1V - LED flashes below this voltage for Hubsan

#define BUZZER_ON_TIME_MS 200
#define BUZZER_OFF_TIME_MS 1000

// hold down switch to toggle from previous protocol
#define BIND_LED_PERIOD_MS 100
#define FLYSKY_LED_PERIOD_MS 2000 // slow flashing
#define HUBSAN_LED_PERIOD_MS 500 // faster flashing

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

#define HUBSAN_RC_MIN 0
#define HUBSAN_RC_MAX 255
#define HUBSAN_RC_NEUTRAL 127 // ???

#define FLYSKY_RC_MIN 1000
#define FLYSKY_RC_MAX 1994
#define FLYSKY_RC_NEUTRAL 1497

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


