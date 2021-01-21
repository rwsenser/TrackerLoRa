// Tracker9X_TX4_GPS, based upon Tracker9X_TX3_M0
// This is for the 9X lora boards
// refactored for the Adafruit M0 Feather
// built with IDE 1.8.7
// Bob Senser, 2020-10-15
//
//  M0 GPS, Version 1
//   
// no for UNO: Board is Arduino/Ger.. UNO  , Programmer is ArduinoISP and set port
// no for Trinked: Board is Adafruit Pro V5 USB, Programmer is USBtinyISP and set
// yes for Adafruit Feather M0, programmer is USBtinyISP, must set Port
//
// Note: uses steps to add Serial2 to Feather M0 (code from serial2test.ino)

// TODOs: 2020-11-10 (changed, 2021-01-13)
// 1)  Add LED blink when transmitting
// 2)  Done -- Find elegant way to obfuscate the GPS data transmitted
//     note than blocks can be lost
//
// Notes:
// 1) This version of tracker accepts serial input from two sources (Telemetry and GPS).
// 2) Telemetry input: pin 11, GPS input: pin RX0
// 3) Actual telemetry input data in bounded with '{' and '}' characters and
//    occurs at regular intervals
// 4) GPS input is recurring and is parsed to obtain needed values
// 5) These inputs are merged to form one output thru the LoRa board
// 6) The Telemetry data is immediately passed to the output, as is
// 7) The GPS output is sent in two cases:
//    When the "@" character is in an input Telemetry line (maybe asyc in time)
//    After a long duration and then it is repeated at some interval
//
// change history:
//
// 2021-01-16: Start full integration of Encode and Encrypt 
//
// 2021-01-15: Added CRC to Encode 
//
// 2021-01-??: Added option to send a regular internal
//             Added another option to do 1) above and then when logging stops, change interval -- done 
//
// 2020-12-03: Added uEncrypt.h and calls to allow encryption
//
// 2020-11-25: Add uEncode.h and calls to allow obfuscation 
//
// 2020-10-19:  Add code to impliment send GPS on "@" char and auto send GPS at internal

// This engages whatever code is ifdef'd for DEBUG_MODE
// #define DEBUG_MODE
//
// catchall buffer size -- set before includes
#define BUFFER_SIZE 64
//
// configure log format 
// #define OBFUSCATE 
//
#include <Arduino.h>   // required before wiring_private.h
#include <wiring_private.h> // pinPeripheral() function
#include <stdlib.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <SparkFunBME280.h>
#ifdef OBFUSCATE
#include "shared/uEncode.h"
#endif

/************ Radio Setup ***************/

#define RF95_FREQ 915.0

// change these  for testing ...
// changed for m0 LaRa
//                    new    old
#define RFM95_INT     3 // 3  // 
#define RFM95_CS      8 // 10  //
#define RFM95_RST     4 // 9  // "A"
#define LED           13
#define TELEM         11  // input
//
// PARMS to SET: //////////////////////////////////////////////////////////
// 0: run GPS serial passthru, 1: Telem serial passthru 2: simple test of BMP280, 3: merge
const int runMode = 3; // 2; // 3;
// 0: off, 1: less verbose, 2: more verbose Serial output
const bool serialLevel = 0;  // 0 needed for battery mode
// 0: no Baro, 1: use Baro with runMode=2
const int useBaro = 1;
// 0: no after Send, 1: send GPS chirps after logging ends
const int afterSendOn = 0;
// 0: no abfuscation, 1: abfuscation on
const int abfuscation = 1;
// GPS autosend interval in millisecs
const unsigned long transCycleMilliSeconds = 60 * 1000; // 60 secs approx
const unsigned long transCycleMilliSeconds2 = 300 * 1000; // 5 mins approx
// END of PARMS to SET
//



int16_t packetnum = 0;  // packet counter, we increment per xmission

BME280 sensor;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


// timestamp for timed sends of data
unsigned long ts;
unsigned long ts2;
unsigned long tsLastSerial;
bool transActive = true;
bool mergeActive = true;
char radiopacket[BUFFER_SIZE + 4];
char radiopacket2[sizeof(radiopacket) + 4];
char sendpacket[sizeof(radiopacket)];
bool waiting = 0;
// bool haveSerialData = false;
bool haveGPSData = false;
bool haveTelemData = false;
bool sendBuffer = false;
int sendBufferLen = 0;
bool forceGPSsend = false;
int len;
long cnt;
char cntBuffer[32];
boolean toggle;
// reading variables
int fieldCnt;
boolean skipLine;
float alt;
float lat;
char latR;
float lng;
char lngR;
float altFT_BMP;
float tempF_BMP;
const int tokenSize = 16;
char token[tokenSize];



// for Serial2
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}


void setup() 
{
  // this pertains to output to Arduino console...
  if (serialLevel > 0) {
    Serial.begin(9600); 
    while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
    Serial.println("Run Mode, Use Baro, AfterSendOn: ");
    Serial.print(runMode);
    Serial.print(" ");
    Serial.print(useBaro);
    Serial.print(" ");     
    Serial.println(afterSendOn);
  }

  Serial1.begin(9600); // GPS
  Serial2.begin(9600); // Telemetry Input 
  
#ifdef OBFUSCATE
  // encode
  Encode enc = Encode(ENCM_CLEAR); // ENCM_CLEAR: clear text, ENCM_ENCRYPT: encrypted   
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

  // Serial2 pin mapings
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);  
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    if (serialLevel > 0) {    
       Serial.println("LoRa radio init failed");
    }   
    while (1);
  }
  if (serialLevel > 0) {
    Serial.println("LoRa radio init OK!");
  }  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  if (!rf95.setFrequency(RF95_FREQ)) {
    if (serialLevel > 0) {
      Serial.println("setFrequency failed");
    }
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23); 

  if (serialLevel > 0) {
    Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");
    if (!transActive) {
      Serial.println("Radio NOT in use!");
    }

  }
#if 1

  if (useBaro) {
    if (serialLevel > 0) {
      Serial.println("Init Sensor!");    
    }    
    Wire.begin();
    sensor.setI2CAddress(0x76);
    if (sensor.beginI2C() == false) //Begin communication over I2C
    {
      if (serialLevel > 0) {
        Serial.println("The sensor did not respond. Please check wiring.");
      }
      while(1); //Freeze
    }
  } else {
    if (serialLevel > 0) {
      Serial.println("No Baro Sensor!");    
    }    
  }
#endif  
  int wTime = 500;
  if (1) { 
    // send 2 dummy transmissions
    strcpy(sendpacket,"Starting 11111");
    len = strlen(sendpacket);
    rf95.send((uint8_t *)sendpacket, len);
    delay(wTime);   
    strcpy(sendpacket,"Starting 222222");
    len = strlen(sendpacket);
    rf95.send((uint8_t *)sendpacket, len);
    delay(wTime);
    if (serialLevel > 0) {
      Serial.println("Sent 2 dummy lines.");
    }               
  }
  // digitalWrite(LED, HIGH);
  // toggle = HIGH;
  cnt = 0;       // something like transaction count
  fieldCnt = 0;     // cnt of field being parsed, reset on new line
  skipLine = false;  // marker for unwanted line, reset on new line
  strcpy(token,""); 
  ts = 0;        // set in other places with millis()
  ts2 = 0;       // ditto... 
  tsLastSerial = 0; // ditto...
}

boolean getBMPdata( float* pAltFT, float* pTempF) {
  float altFT = sensor.readFloatAltitudeFeet(); 
  float tempF = sensor.readTempF();
  if (serialLevel == 2) {
    Serial.print("altFT: "); Serial.print(altFT); Serial.print(" tempF: "); Serial.println(tempF);
  }
  *pAltFT = altFT;
  *pTempF = tempF;
  return true;
}

// for lat/log:  DD = d + (min/60) + (sec/3600)
// here we have only degs and mins (no secs, done as fractional minutes)
float toDegs( float _v) {
  int d = ((int) _v) / 100;   // degrees
  float m = (_v - (d * 100)) / 60.0; // minutes expressed in degrees
#if 0  
  Serial.print("DEBUG:");
  Serial.print(" ");
  Serial.print(_v);
  Serial.print(" ");
  Serial.print(d); 
  Serial.print(" "); 
  Serial.println(m);        
  delay(1000);
#endif  
  return (d + m); 
}

boolean processField(int fCnt, char * t, boolean skipLine) {
  boolean skip = skipLine;
  if (!skip && (strlen(t) > 0)) {
    // Serial.print("Field: "); Serial.print(fCnt); Serial.print(" :: "); Serial.println(t);
    if (fCnt == 0 && (strcmp(t,"$GPGGA") != 0)) {  // we only want to use GPGGA records
      skip = true;
    } else {
      // process the field by its number

      switch (fCnt) {
        case (2): // lat
          lat = atof(t);
          // fix 2020-07-16
          lat = toDegs(lat);
          if (serialLevel == 2) { Serial.print(" lat: "); Serial.println(lat); }
          break;
        case (3): // lat region
          latR = t[0];
          if (serialLevel == 2) { Serial.print(" lat r: "); Serial.println(t); }
          break;        
        case (4): // longitude
          lng = atof(t);
          // fix 2020-07-16
          lng = toDegs(lng);
          if (serialLevel == 2) { Serial.print(" lng: "); Serial.println(lng); }
          break;       
        case (5): // longitude region
          lngR = t[0];
          if (serialLevel == 2) { Serial.print(" lng r: "); Serial.println(t); }
          break;        
        case (9): // altitude
          alt = atof(t) * 3.28084; // in feet
          if (serialLevel == 2) { Serial.print(" alt: "); Serial.println(alt); }
          break;        
      }
    }
  }
  return skip;
}
//
// RT ERROR
//
void rtError(int _d) {
  while (1) {
    if (toggle) {
      digitalWrite(LED, HIGH);            
    } else {
      digitalWrite(LED, LOW);              
    }
    toggle = !toggle;
    delay(_d); 
  }   
}

//
// MODE 0
//
void mode0() {
  if (Serial1.available()>0) {
    haveGPSData = true;
    len = Serial1.readBytes(radiopacket, sizeof(radiopacket)-1);
    radiopacket[len] = 0;  
    // note: likely nothing is listening to these Serial outputs
    if (serialLevel > 0) { 
      Serial.print("GPS Transmitting: "); Serial.println(radiopacket);
    }
    sendBuffer = true;
    sendBufferLen = len;
  }     
  return;
}
void mode1() {
  if (Serial2.available()>0) {
    haveTelemData = true;
    len = Serial2.readBytes(radiopacket, sizeof(radiopacket)-1);
    radiopacket[len] = 0;  
    // note: likely nothing is listening to this Serial output
    if (serialLevel > 0) { 
      Serial.print("Telem Transmitting: "); Serial.println(radiopacket);
    }
    sendBuffer = true;
    sendBufferLen = len;    
  }     
  return;
}
//
// MODE 2
//
void mode2() {
  // input serial port or create test data (runMode == 1)
  delay(5000);
  float altFT = sensor.readFloatAltitudeFeet(); 
  float tempF = sensor.readTempF();
  char line[64];
  if (serialLevel > 0) {
    Serial.print(" Alt: ");
    Serial.print(altFT, 1);
    Serial.print(" Temp: ");
    Serial.println(tempF, 2);
  }
  int iAltFT = altFT * 100;
  int iTempF = tempF * 100;
  sprintf(line," A: %d.%d T: %d.%d", iAltFT/100, iAltFT%100, iTempF/100, iTempF%100);      
  haveGPSData = true;
  ltoa(cnt,cntBuffer,10);
  strcpy(radiopacket,"LoRa Baro:");
  strcat(radiopacket,cntBuffer);
  strcat(radiopacket,line);
  len = strlen(radiopacket);
  if (serialLevel > 0) {
    Serial.print("Baro Data: "); Serial.println(radiopacket);
  }      
  sendBuffer = true;
  sendBufferLen = len;  
  cnt++;
  return;
}
//
// MODE 3
//
void mode3() {
  // merge, parse GPS data and send
  // 4 parts:  1, 2 then 3 or 4
  //   1:  Get updated GPS data
  //   2:  pick which source (Telem or GPS)
  //   3:  Processes Telem data
  //   4:  Format the GPS packet
  // vars: token, alt, lat, lng
  // todo, add timer check
  // Part 1: process input data, could straddle packets from GPS
  //         a line to cr/lf might not equal a packet!
  //
  // MODE 3, part 1: GPS input 
  //
  if (Serial1.available()>0) {
    haveGPSData = true;
    len = Serial1.readBytes(radiopacket, sizeof(radiopacket)-1);
    radiopacket[len] = 0;  
    for (int k=0; k < strlen(radiopacket); k++) {
      char c = radiopacket[k];
      // Serial.print("c: "); Serial.print(c); Serial.print(" :: "); Serial.println(((int) c));
      if (c == ',') {
        skipLine = processField(fieldCnt, token, skipLine);
        fieldCnt++;  
        strcpy(token,"");          
      } else if (c == '\n' || c == '\r') {
        skipLine = processField(fieldCnt, token, skipLine);
        fieldCnt = 0;
        strcpy(token,""); 
        skipLine = false;           
      } else {  // add char to token
        char c2[2];
        c2[0] = c;
        c2[1] = 0;
        if (strlen(token) < tokenSize-1) { // prevent overflow
          strcat(token,c2);
        }   
      }
    }
    // Serial.print("Transmitting: "); Serial.println(radiopacket);      
  }
  //
  // MODE 3, part 2: choose between GPS immediate send and Telem
  if (Serial2.available()>0) {
    len = Serial2.readBytes(radiopacket, sizeof(radiopacket)-1);
    radiopacket[len] = 0;  
    // Serial.print("DEBUG: ");  Serial.println(radiopacket); 
    // handle '@' input which is "send GPS data" now  
    if(strchr(radiopacket, '@') != NULL) {  // '@' means send GPS data now, no wait!
      // send GPS data!!
#if 0      
      haveTelemData = false;
      forceGPSsend = true;
#endif            
    } else {  
      // otherwise process normal telem data   
      haveTelemData = true;
      // MODE 3, part 3: Format Telem data    
      // note: likely nothing is listening to this Serial output, maybe the IDE console
      if (serialLevel > 0) { 
        Serial.print("Telem Data: "); Serial.println(radiopacket);
      }
      sendBuffer = true;
      sendBufferLen = len;      
      ts2 = millis();  // if we send actual data, reset 2nd timer
      tsLastSerial = millis();  // if we send actual data, reset lastSerial timer     
    }     
  } 
  // consider sending GPS data instead
  if (haveTelemData == false) { 
#ifdef DEBUG_MODE
    {
      static int cnt = 0;
      Serial.print("* ");
      if (++cnt > 30) {
        Serial.println(" ");
        cnt = 0;    
      }
    }
#endif          
    // MODE 2, part 4: Format GPS data, 
    // possibly send even if no new data!
    // Serial.println("DEBUG, send GPS??");
    // delay(200);
    bool sendGPS = false;
    char type = '?'; 
    unsigned int mils = millis();
    // Serial.print(mils); Serial.print(" "); // , ts, ts2);
    // Serial.print(ts); Serial.print(" ");
    // Serial.println(ts2); 
    // timer 2 
    if ( (ts2 + transCycleMilliSeconds2  < mils) &&
          afterSendOn ) {
      sendGPS = true;
      type = '9';
      // timer 1
    } else if ((ts + transCycleMilliSeconds < mils)
           && ((mils - tsLastSerial) < transCycleMilliSeconds) ) {
      sendGPS = true;
      type = '1';
      // forced            
    } else if (forceGPSsend) { // just do it!
      sendGPS = true;
      type = '7';
    }
    if (sendGPS) {
      ts = millis();
      ts2 = millis();
      // create radiopacket
      cnt++;
      if (useBaro) {  
        getBMPdata(&altFT_BMP, &tempF_BMP);
      } else {
        altFT_BMP = -1.0;
        tempF_BMP = -1.0;
      }
      if (lngR == 'W' && lng > 0.0) lng *= -1.0; 
      if (latR == 'S' && lat > 0.0) lat *= -1.0;   
    
      if (useBaro) {
        sprintf(radiopacket, "%c%d,%ld,%f,%f,%.2f,%.2f,%.2f",
                type, cnt, (ts / 1000), lat, lng, alt, altFT_BMP, tempF_BMP);
      } else {
        sprintf(radiopacket, "%c%d,%ld,%f,%f,%.2f",type, cnt, (ts / 1000), lat, lng, alt);  
      }

      if (serialLevel > 0) { 
        Serial.print("GPS Data: "); Serial.println(radiopacket);
      }  
      sendBuffer = true;
      len = strlen(radiopacket);   
      if (abfuscation) {
#ifdef OBFUSCATE                      
        Encode encT = Encode(ENCM_ENCRYPT); // this sets encryp mode ... ENCM_CLEAR   or ENCM_ENCRYPT
        unsigned char buffer2[BUFFER_SIZE];
        int used = -1;  
        ENC_RET ret = encT.encode((unsigned char *) radiopacket, len, buffer2, sizeof(buffer2), &used);
        Serial.print("DEBUG encode: ");
        Serial.println(ret);
        if (ret == ENC_OK) {
          memcpy(radiopacket, buffer2, used);
          len = used;      
        } else {
          memcpy(radiopacket, "ERR!", 4);
          len = 4;
        }
        Serial.print("to send: ");
        for (int i=0; i < len; i++) {
          int v = radiopacket[i];
          Serial.print(v < 16 ? "0" : "");
          Serial.print(v, HEX);
          Serial.print(" ");
        }        
        Serial.println("");
#endif        
      }     
    } else {
      // haveSerialData = false;
      sendBuffer = false;    
    }
  }      
  return;
}

void loop() {
#ifdef DEBUG_MODE
  delay(100);
#endif  
 
  haveTelemData = false;
  haveGPSData = false;
  sendBuffer = false;
  forceGPSsend = false;  
  switch (runMode) {
    case 0: mode0(); break;      
    case 1: mode1(); break;
    case 2: mode2(); break;
    case 3: mode3(); break;    
    default: /* Huh? */
      rtError(500);
  }
  // we like to flash, but not with runmode 3
  if ((haveGPSData || haveTelemData) && runMode != 3) {    
    if (toggle) {
      digitalWrite(LED, HIGH);            
    } else {
      digitalWrite(LED, LOW);              
    }
    toggle = !toggle;
  }  
  // LoRa Transmission is here:       
  // Send a message!
  if (transActive && sendBuffer) {
    if (len > (RH_RF95_MAX_MESSAGE_LEN - 1)) {
      len = (RH_RF95_MAX_MESSAGE_LEN - 1); // around 250
    }  
    // this odd-looking code is poor man's double buffering :)
    if (waiting == 1) {
      rf95.waitPacketSent(); 
    }
    waiting = 0;          
    memcpy(sendpacket,radiopacket,len);
    rf95.send((uint8_t *)sendpacket, len);
    waiting = 1;
    if (serialLevel > 0) {          
      Serial.print("Tran>>> "); Serial.println(radiopacket);
      Serial2.println(radiopacket);
    }    
  } 
}
/* end of code */
