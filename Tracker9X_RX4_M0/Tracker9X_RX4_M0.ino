// Traker9X_RX4_M0 -- based on Tracker9X_RX3_M0/Track9X_RX1
// Bob Senser, 2021-01-18
// used with RF9X
//
// for RFM95(X)
// 
// 2021-01-18:
// * Add the obfuscate capability (matched to Tracker9x_TX4_GPS)
//
// 2020-10-26:
// * For feather M0 use
// * Added 3 status LEDs (LoRa OK, SD OK, Receive (blinks @ read)
// * Added buzzer
//
// catchall buffer size -- set before includes
#define BUFFER_SIZE 64
// configure log format 
// #define OBFUSCATE 

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
// #include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#ifdef OBFUSCATE
#include "shared/uEncode.h"
  // encode
  Encode enc = Encode(ENCM_ENCRYPT); // ENCM_CLEAR: clear text, ENCM_ENCRYPT: encrypted   
#endif
#define SD_PRESENT

/************ Radio Setup ***************/
#define RF95_FREQ 915.0

// changed for m0 LaRa
//                    new    old
#define RFM95_INT     3 // 3  // 
#define RFM95_CS      8 // 10  //
#define RFM95_RST     4 // 9  // "A"
#define LED           13

// manage multiple spi devices
// radio on above
#define SD_CS 15
#define BUZ_PIN 14

// status LEDs (two color)
// 3 LEDS, 2 pins per LED
const int statLEDs[] = {10, 9, 6, 5, 21, 20};
const int LEDoff = 0;
const int LEDred = 1;
const int LEDgreen = 2;

// ********** config:
const bool testMode    = false;
const bool SDcardOn    = true;
const bool logPrefixOn = true;
const bool serialLevel = false; // true; // false;
// ********** end of config
bool serialOutOn = false;
// watchdog timer
const long watchLimit = 60000; // 1 minute
long clockBasis = 0;
// buzzer 'thread' controller
long buzStop = 0;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// LCD
// LiquidCrystal_I2C lcd(0x27, 16, 2);

int packetnum = 0;  // packet counter, we increment per xmission
const char OK[] = "OK";
const char FAILED[] = "FAILED";
const char SDOFF[] = "; SD off";
const char SDON[] = "; SD on";
const char RFMON[] = "RFM on";
const char RADIOERR[] = "Radio Err!";
const char RADIORST[] = "Radio Rst!";
const char TSTDATA0[] = "tData0!";
const char TSTDATA1[] = "tData1!";

void setLED(int _l, int _v) {
  int k = (_l * 2);
  switch (_v) {
    case (LEDoff): 
      digitalWrite(statLEDs[k], HIGH);
      digitalWrite(statLEDs[k+1], HIGH);
      break;      
    case (LEDred):
      digitalWrite(statLEDs[k], HIGH);
      digitalWrite(statLEDs[k+1], LOW);
      break;       
    case (LEDgreen):
      digitalWrite(statLEDs[k], LOW);
      digitalWrite(statLEDs[k+1], HIGH);
      break;       
  }
}

void setup() 
{
  bool err = 0;
  if (serialLevel > 0) {
    Serial.begin(9600); 
    while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
    Serial.print("Tacker9X_RX3_M0 running: testMode is ");
    Serial.println(testMode); 
    serialOutOn = true;       
  }  
  //manage multiple spi devices
  pinMode(SD_CS, OUTPUT);   
  pinMode(RFM95_CS, OUTPUT); 
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RFM95_CS, HIGH);       
  pinMode(RFM95_RST, OUTPUT);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  // init buzzer
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(BUZ_PIN, HIGH);
  // simple LED
  pinMode(LED, OUTPUT);  
  digitalWrite(LED, HIGH);
  // init status LEDS
  for (int k=0; k < 6; k++) {
    pinMode(statLEDs[k], OUTPUT);  
    digitalWrite(statLEDs[k], HIGH);
  } 

  setLED(0,LEDred); 
  setLED(1,LEDred); 
  setLED(2,LEDred); 

#ifdef OBFUSCATE
// TESTS
#if 0
  // test encode, echo back
  //enc_test("123,456,-12.034");
  enc_test("11,71,0.000000,0.000000,0.00,6198.62,71.13");
  while(1) { }
#endif

#if 0
  // text encrypt
  ecy_test("BLUE FROG", "Donald Loser");
  while (1) { }
#endif  
#endif
      
  if (!rf95.init()) {
    if (serialOutOn) {Serial.println("LoRa radio init failed");}
    err = 1;
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    if (serialOutOn) {Serial.println("setFrequency failed");}
    err = 1;
  } else {
    if (serialOutOn) {
      Serial.print("RFM95 radio @");
      Serial.print((int)RF95_FREQ);
      Serial.println(" MHz");
    }
  }
  if (err == 0) {
    setLED(0,LEDgreen);
  }     
  // see if the card is present and can be initialized:
  // must init first ....
#ifdef SD_PRESENT 
  if (SDcardOn) {
    if (serialOutOn) {Serial.print("Card ");} 
    if (!SD.begin(SD_CS)) {
      if (serialOutOn) {Serial.println(FAILED);}
      err = 1;
    } else {
      if (serialOutOn) {Serial.println(OK);}
    }
  }
  if (err == 0) {
    setLED(1,LEDgreen);
  } 
#else
 setLED(1,LEDred);
#endif  
  if (err == 1) {
    if (serialOutOn) {Serial.print("Failed to Init!");}     
    while (1) { }
  }

  Serial.print(RFMON);
  if (SDcardOn == 0) {
   if (serialOutOn) {Serial.println(SDOFF);} 
  } else {
    if (serialOutOn) {Serial.println(SDON);}    
  }
  // watchdog timer
  clockBasis = millis();

  // buzzzz
  buzOn(200);
  // digitalWrite(BUZ_PIN, LOW);
  // delay(200);
  // digitalWrite(BUZ_PIN, HIGH); 
  if (err == 0) {
    setLED(2,LEDgreen);
  }    
}

void buzOn(int mSec) {
  buzStop = mSec + millis();
  setLED(2,LEDred);  
  digitalWrite(BUZ_PIN, LOW);
}
void buzOff() {
  if (millis() > buzStop) {
    digitalWrite(BUZ_PIN, HIGH); 
    buzStop = 0; 
    setLED(2,LEDgreen);     
  }
  return;
}

void loop() {
  // give buzzer thread a shot
  buzOff();
  if (testMode) {
      // lcd.clear(); 
      const char * ptr;
      if (packetnum % 2 == 0) {
        ptr = TSTDATA0;
      } else {
        ptr = TSTDATA1;
      }
      // lcd.print(ptr);
      if (serialOutOn) {Serial.print(ptr);}
      if (serialOutOn) {Serial.print("\r");}
#ifdef SD_PRESENT        
      if (SDcardOn) {
         File dataFile = SD.open("datalog.txt", FILE_WRITE);
         if (dataFile) {
           dataFile.println((char*)ptr);
           dataFile.print("\r");           
           dataFile.close();
         }
      }  
#endif        
      packetnum++;
      delay(1000);     
  } else {
    // RUN MODE ....
     if (rf95.available()) {
       // Should be a message for us now   
       uint8_t buf[BUFFER_SIZE]; // [RH_RF95_MAX_MESSAGE_LEN + 8];
       uint8_t buf2[BUFFER_SIZE];
       uint8_t buf3[BUFFER_SIZE];       
       uint8_t len = sizeof(buf);   
       // get signal strength               
       sprintf((char *) buf3,"|%d", rf95.lastRssi()); 
       // if (serialOutOn) {Serial.print((char *) buf3);}        
       int len2 = 0; // strlen((char *) buf);
       if (logPrefixOn) {
         len2 = strlen((char *) buf3);
       }
       
       if (rf95.recv(buf, &len)) {
#ifdef OBFUSCATE
         int used;
         bool is_ASCII = true;
         int chk_len = len;
         if (chk_len > 8) chk_len = 8; 
         for (int i=0; i < chk_len; i++) {
           if (!isprint(buf[i])) {
             is_ASCII = false;
           }
         } 
         if (!is_ASCII) { 
           // binary in buf
           ENC_RET ret = enc.decode(buf, len, buf2, sizeof(buf2), &used);
           if (ret == ENC_OK) {
             len = used;
             memcpy(buf, buf2, len); // brute force!
           } else {
             strcpy((char *) buf, "ENC FAILED!");
             len = strlen((char *) buf);
           }
         }      
#endif        
         buf[len] = 0;  
         // clean out any non-ASCII
         for (int i=0; i < len; i++) {
           if (!isprint(buf[i])) buf[i] = '.';
         }
         // add signal strength
         strcat((char *) buf, (char *) buf3);   // is this still under 64???
         if (serialOutOn) {
           Serial.print((char *) buf);  
           Serial.print("\n\r");
         }    
         // SD output
#ifdef SD_PRESENT        
         if (SDcardOn) {
           File dataFile = SD.open("datalog.txt", FILE_WRITE);
           if (dataFile) {
             dataFile.println((char*)buf);
             dataFile.close();
           }
         }  
#endif       
         // buzzzz
         buzOn(75);           
         len = strlen((char *) buf); // now len of all used space
         // no longer a c string, padded instead...
#if 0       
         // lcd code -- disabled  
         for (int i=len; i < sizeof(buf); i++ ) { buf[i] = ' '; }      
           int usable = 17;
           // lcd.clear(); 
           // lcd.print((char *)prefix);
           if (len < usable) {     
             lcd.print((char*)buf);
           } else {
             lcd.setCursor(0,1);
             lcd.print((char *) &buf[(usable-1)]);
             buf[16]= 0; // bad way ...
             lcd.setCursor(0,0);
             lcd.print((char *) buf);
           }    
         }   
#endif             
       } else {
         if (serialOutOn) {
           Serial.print("Rec ");
           Serial.println(FAILED);
         } 
       }
    } // of if available
  } // of mode
} // of loop
