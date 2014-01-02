
/*
  Standalone Tx adapter version by Prof. G.K. Egan (gke) 2013.
 
  SPI routines adapted from UAVXPIC and Midelic's FlySky adapter.
 
  Deviation is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License 
  if not, see <http://www.gnu.org/licenses/>.
 
*/

#include <Arduino.h>

#define USE_PORT_DIRECT
#if defined(USE_PORT_DIRECT)

// CAUTION: magic only works for D0-D7

#define  CS_HI() PORTD |= (1<<CS_PIN)
#define  CS_LO() PORTD &= ~(1<<CS_PIN)

#define  SCK_HI() PORTD |= (1<<SCLK_PIN)
#define  SCK_LO() PORTD &= ~(1<<SCLK_PIN)
#define  SDIO_HI() PORTD |= (1<<SDIO_PIN)
#define  SDIO_LO() PORTD &= ~(1<<SDIO_PIN)
//
#define  SDIO_1() (PIND & (1<<SDIO_PIN)) == (1<<SDIO_PIN)
#define  SDIO_0() (PIND & (1<<SDIO_PIN)) == 0x00

#else

#define  CS_HI() digitalWrite(CS_PIN, HIGH)
#define  CS_LO() digitalWrite(CS_PIN, LOW)
//
#define  SCK_HI() digitalWrite(SCLK_PIN, HIGH)
#define  SCK_LO() digitalWrite(SCLK_PIN, LOW)
#define  SDIO_HI() digitalWrite(SDIO_PIN, HIGH)
#define  SDIO_LO() digitalWrite(SDIO_PIN, LOW)

#define  SDIO_1 digitalRead(SDIO_PIN) == 0x20
#define  SDIO_0 digitalRead(SDIO_PIN) == 0x00

#endif

#define SPI_DELAY() delayMicroseconds(1) //__asm__ __volatile__("nop")

void a7105Reset(void) {

  a7105WriteReg(0x00, 0x00);
  delay(1);
} // a7105Reset

void a7105Setup(void) {

  pinMode(SDIO_PIN, OUTPUT); 
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  CS_HI();
  SDIO_HI();
  SCK_LO();

  a7105Reset(); 
} // a7105Setup

void a7105WriteID(uint32_t id) {
  CS_LO();
  a7105Write(A7105_06_ID_DATA);
  a7105Write((id >> 24) & 0xff); 
  a7105Write((id >> 16) & 0xff);
  a7105Write((id >> 8) & 0xff);
  a7105Write((id) & 0xff);
  CS_HI();
} //a7105WriteID

void a7105ReadID(void) {
  uint8_t i;

  CS_LO();
  a7105Write(0x40 | A7105_06_ID_DATA);
  if (DEBUG)
    for(i = 0; i < 4; i++)
      checkID[i] = a7105Read();
  CS_HI();
} // a7105ReadID

void a7105Write(uint8_t c) {  
  uint8_t n=8; 

  SCK_LO();
  SDIO_LO();
  while(n--) {
    if(c & 0x80)
      SDIO_HI();
    else 
      SDIO_LO();
    SCK_HI();
    SPI_DELAY();
    SCK_LO();
    c = c << 1;
  }
  SDIO_HI();
} // a7105Write

void a7105WriteReg(uint8_t a, uint8_t d) {
  CS_LO();
  a7105Write(a); 
  SPI_DELAY();
  a7105Write(d);  
  CS_HI();
} // a7105WriteReg 

void a7105WriteData(uint8_t *buf, uint8_t len, uint8_t chan) {
  uint8_t i;

  // pinMode(SDIO, OUTPUT);
  // digitalWrite(SDIO, LOW);
  CS_LO();
  a7105Write(A7105_RST_WRPTR); 
  a7105Write(A7105_05_FIFO_DATA); 
  for (i = 0; i < len; i++)
    a7105Write(buf[i]); 
  SCK_LO();
  CS_HI();
  //pinMode(SDIO, INPUT);

  a7105WriteReg(0x0f, chan); // set the channel

  CS_LO();
  a7105Write(A7105_TX);
  //digitalWrite(SCK, LOW);
  CS_HI();
} // a7105WriteData

static void a7105CRCUpdate(uint8_t len) {
  int16_t sum = 0;
  uint8_t i;

  for (i = 0; i < (len-1); i++)
    sum += packet[i];
  packet[len-1] = (256 - (sum % 256)) & 0xff;
} // a7105a7105CRCUpdate


uint8_t a7105Read(void) {
  uint8_t d = 0;
  uint8_t i;

  pinMode(SDIO_PIN, INPUT);
  //SDIO_HI();
  for(i = 0; i < 8; i++) {                    
    if(SDIO_1()) 
      d = (d << 1) | 0x01;
    else
      d = d << 1;
    SCK_HI();
    SPI_DELAY();
    SCK_LO();
    SPI_DELAY();
  }
  pinMode(SDIO_PIN, OUTPUT);
  return (d);
} // a7105Read 

uint8_t a7105ReadReg(uint8_t a) { 
  uint8_t d;

  CS_LO();
  a7105Write(0x40 | a);
  
  delayMicroseconds(4);
  
  d = a7105Read();  
  CS_HI();

  return(d); 
} // a7105ReadReg

void a7105ReadData(uint8_t *buf, uint8_t len) {
  uint8_t i;

  a7105Strobe(A7105_RST_RDPTR); 
  for(i = 0; i < len; i++) 
    buf[i] = a7105ReadReg(A7105_05_FIFO_DATA);
} // a7105ReadData

boolean a7105FlagDone(uint8_t a) {
  uint32_t TimeoutmS = millis() + 500;

  while((millis() < TimeoutmS) && (a7105ReadReg(a) != 0));

  return  (millis() < TimeoutmS);
} // a7105FlagDone

void a7105Strobe(uint8_t state) {

  CS_LO();
  a7105Write(state);
  CS_HI();
} // a7105Strobe

void a7105SetPower(uint8_t p) {
  /*
   Power amp is ~+16dBm so:
   TXPOWER_100uW  = -23dBm == PAC=0 TBG=0
   TXPOWER_300uW  = -20dBm == PAC=0 TBG=1
   TXPOWER_1mW    = -16dBm == PAC=0 TBG=2
   TXPOWER_3mW    = -11dBm == PAC=0 TBG=4
   TXPOWER_10mW   = -6dBm  == PAC=1 TBG=5
   TXPOWER_30mW   = 0dBm   == PAC=2 TBG=7
   TXPOWER_100mW  = 1dBm   == PAC=3 TBG=7
   TXPOWER_150mW  = 1dBm   == PAC=3 TBG=7
   */

  const uint8_t pac[] = {
    0,0,0,0,1,2,3,3,0
  };
  const uint8_t tbg[] = {
    0,1,2,4,5,7,7,7,0
  };

  p = constrain(p, 0, 8);  
  a7105WriteReg(A7105_28_TX_TEST, (pac[p] << 3) | tbg[p]);
  
} // a7105SetPower















