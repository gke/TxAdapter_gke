
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
 
// Battery 

#define LVC_DELAY_TIME_S 2 // autoland after this delay in seconds
#define LVC_WARNING_PERCENT 90 // scales down desired throttle "suddenly" to this percentage when low volts reached
#define LVC_LANDING_TIME_S 5 // time for throttle to drop to zero

#if !defined(LVC_LIMIT)
#define LVC_LIMIT 0
#endif

#if !defined(LVC_WARNING_PERCENT)
#define LVC_WARNING_PERCENT 80 // scales down desired throttle "suddenly" to this percentage when low volts reached
#endif

#if !defined(LVC_DELAY_TIME_S)
#define LVC_DELAY_TIME_S 3
#endif

#if !defined(LVC_LANDING_TIME_S)
#define LVC_LANDING_TIME_S 3
#endif

#define LVC_TRIGGER_TIME_S 2

#define LVC_UPDATE_MS 25
#define LVC_BUCKET_TRIG ((LVC_TRIGGER_TIME_S*1000)/LVC_UPDATE_MS)

#define LVC_TEMP ((1024L*LVC_UPDATE_MS)/(LVC_LANDING_TIME_S*1000L))
#if (LVC_TEMP>0)
#define LVC_DECAY_STEP LVC_TEMP
#else
#define LVC_DECAY_STEP 1
#endif

void checkBattery(void) { // bucket suggestion due to vlad_vy
#if (LVC_LIMIT > 0)
  enum lvcStates {
    Start = 0, Monitor, Warning, Wait, Land
  };
  uint32_t NowmS;

  static uint8_t lvcState = Monitor;
  static uint32_t lvcUpdatemS = millis();
  static uint32_t lvcTimeoutmS = millis();
  static int8_t bucket = LVC_BUCKET_TRIG;
  static uint32_t v = (200<<2); // set high for startup
  static uint16_t newv;

  NowmS = millis();
  if (NowmS > lvcUpdatemS ) {
    lvcUpdatemS = NowmS + LVC_UPDATE_MS;

    switch (lvcState) {
    case Monitor:
      Alarm(false);
      if (batteryVolts <= LVC_LIMIT )
        lvcState = Warning;
      else
        if (bucket < LVC_BUCKET_TRIG) 
          bucket += 2;
      break;
    case Warning:
      Alarm(true);
      if (batteryVolts > LVC_LIMIT ) {
        throttleLVCScale = 1024;
        lvcState = Monitor;   
      } 
      else
        if (bucket <= 0) {
          throttleLVCScale = ((LVC_WARNING_PERCENT * 1024L)/100);
          lvcTimeoutmS = NowmS + (LVC_DELAY_TIME_S * 1000);  
          lvcState = Wait;
        }
        else
          bucket--;
      break;
    case Wait:
      if (NowmS > lvcTimeoutmS)
        lvcState = Land;    
      break;
    case Land:
      if (throttleLVCScale >= LVC_DECAY_STEP)  
        throttleLVCScale -= LVC_DECAY_STEP;
      else 
        disableThrottle = true;  
      break;
    } // switch 
  }
#else
  throttleLVCScale = 1024;
#endif
} // checkBattery

