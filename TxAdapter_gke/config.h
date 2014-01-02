#define Limit1(i,l) (((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))
#define Sign(n) ((n<0) ? -1 : 1)

#define CS_PIN 2 
#define PPM_PIN 3 
#define SCLK_PIN 4
#define SDIO_PIN 5
#define HUBSAN_PIN 6

#define LED_PIN 13

#define LED_ON() digitalWrite(LED_PIN, HIGH)
#define LED_OFF() digitalWrite(LED_PIN, LOW)

#define USING_HUBSAN false // digitalRead(PROTOCOL_PIN)

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

enum ProtoCmds {
    PROTOCMD_INIT,
    PROTOCMD_DEINIT,
    PROTOCMD_BIND,
    PROTOCMD_CHECK_AUTOBIND,
    PROTOCMD_NUMCHAN,
    PROTOCMD_DEFAULT_NUMCHAN,
    PROTOCMD_CURRENT_ID,
    PROTOCMD_SET_TXPOWER,
    PROTOCMD_GETOPTIONS,
    PROTOCMD_SETOPTIONS,
    PROTOCMD_TELEMETRYSTATE,
};


