// Tracker9X_TX3-M0, based upon Tracker9X_TX2
// This is for the 9X lora boards
// refactored for the Adafruit M0 Feather
// built with IDE 1.8.7
// Bob Senser, 06/20/2020
//
//  M0 Version 1
//   
// no for UNO: Board is Arduino/Ger.. UNO  , Programmer is ArduinoISP and set port
// no for Trinked: Board is Adafruit Pro V5 USB, Programmer is USBtinyISP and set
// yes for Adafruit Feather M0, programmer is USBtinyISP, must set Port
//
// 06/20/2020: First usable version completed
// 06/11/2020: Worked with actual GPS!
// 06/07/2020: Begin adding actual run mode that parses serial input data
// 06/06/2020: Add serial passthru runMode that ignores the BMP280 data
// 06/02/2020: Add BMP/BME 280 support
// 05/16/2020: Create from old versions(s)
//
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <SparkFunBME280.h>
/************ Radio Setup ***************/

#define RF95_FREQ 915.0

// change these  for testing ...
// changed for m0 LaRa
//                    new    old
#define RFM95_INT     3 // 3  // 
#define RFM95_CS      8 // 10  //
#define RFM95_RST     4 // 9  // "A"
#define LED           13

BME280 sensor;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission
// 0: run serial passthru, 1: simple test of BMP280, 2: merge
int runMode = 2;
// 0: less verbose, 1:more verbose Serial output
bool serialLevel = 0;
int transActive = 1;
char radiopacket[68];
char radiopacket2[sizeof(radiopacket) + 4];
char sendpacket[sizeof(radiopacket)];
bool waiting = 0;
bool haveSerialData = false;
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
// timestamp for timed sends of data
unsigned long ts;
const unsigned long transCycleMilliSeconds = 20000;

void setup() 
{
  Serial.begin(9600); 
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  Serial1.begin(9600);

  Serial.print("Run Mode: ");
  Serial.println(runMode);
  
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23); 
  
  Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");
  if (transActive == 0 && runMode == 0 ) {
    Serial.println("Radio NOT in use!");
  }
  Serial.println("Init Sensor!");
#if 1
  Wire.begin();
  sensor.setI2CAddress(0x76);
  if (sensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
#endif  
  // digitalWrite(LED, HIGH);
  // toggle = HIGH;
  cnt = 0;       // something like transaction count
  fieldCnt = 0;     // cnt of field being parsed, reset on new line
  skipLine = false;  // marker for unwanted line, reset on new line
  strcpy(token,""); 
  ts = 0;        // set in other places with millis()  
}

boolean getBMPdata( float* pAltFT, float* pTempF) {
  float altFT = sensor.readFloatAltitudeFeet(); 
  float tempF = sensor.readTempF();
  if (serialLevel) {
    Serial.print("altFT: "); Serial.print(altFT); Serial.print(" tempF: "); Serial.println(tempF);
  }
  *pAltFT = altFT;
  *pTempF = tempF;
  return true;
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
          if (serialLevel) { Serial.print(" lat: "); Serial.println(lat); }
          break;
        case (3): // lat region
          latR = t[0];
          if (serialLevel) { Serial.print(" lat r: "); Serial.println(t); }
          break;        
        case (4): // longitude
          lng = atof(t);
          if (serialLevel) { Serial.print(" lng: "); Serial.println(lng); }
          break;       
        case (5): // longitude region
          lngR = t[0];
          if (serialLevel) { Serial.print(" lng r: "); Serial.println(t); }
          break;        
        case (9): // altitude
          alt = atof(t) * 3.28084; // in feet
          if (serialLevel) { Serial.print(" alt: "); Serial.println(alt); }
          break;        
      }
    }
  }
  return skip;
}

void loop() {
    haveSerialData = false;
    // input serial port or create test data (runMode == 1)
    //
    // MODE 1
    //
    if (runMode == 1) {
      delay(5000);
      float altFT = sensor.readFloatAltitudeFeet(); 
      float tempF = sensor.readTempF();
      char line[64];
      Serial.print(" Alt: ");
      Serial.print(altFT, 1);
      Serial.print(" Temp: ");
      Serial.println(tempF, 2);
      int iAltFT = altFT * 100;
      int iTempF = tempF * 100;
      sprintf(line," A: %d.%d T: %d.%d", iAltFT/100, iAltFT%100, iTempF/100, iTempF%100);      
      haveSerialData = true;
      ltoa(cnt,cntBuffer,10);
      strcpy(radiopacket,"LoRa Radio:");
      strcat(radiopacket,cntBuffer);
      strcat(radiopacket,line);
      len = strlen(radiopacket);
      Serial.print("Test Data Transmitting: "); Serial.println(radiopacket);      
      cnt++;
    } else if (runMode == 2) {
      // parse GPS data
      // 2 parts:
      //   1:  Get updated GPS data
      //   2:  Format and send packet
      // vars: token, alt, lat, lng
      // todo, add timer check
      // Part 1: process input data, could straddle packets from GPS
      //         a line to cr.lf might not equal a packet!
      //
      // MODE 2, part 1
      //
      if (Serial1.available()>0) {
        haveSerialData = true;
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
    } // note the Serial1 here...
    // pass thru 
    else if (Serial1.available()>0) {
        haveSerialData = true;
        len = Serial1.readBytes(radiopacket, sizeof(radiopacket)-1);
        radiopacket[len] = 0;  
        // note: likely nothing is listening to these Serial outputs 
        Serial.print("Transmitting: "); Serial.println(radiopacket);
    }    
    // LoRa Transmission is here:
    //
    // MODE 2, part 2
    //
    if (runMode == 2) {
      if (ts + transCycleMilliSeconds < millis()) {
        ts = millis();
        // create radiopacket
        cnt++;  
        getBMPdata(&altFT_BMP, &tempF_BMP);    
        if (lngR == 'W') lng *= -1.0; 
        if (latR == 'S') lat *= -1.0;  
        lng = lng / 100.0; // scaling
        lat = lat / 100.0; // scaling
        sprintf(radiopacket, "%d,%ld,%f,%f,%.2f,%.2f,%.2f\n", cnt, (ts / 1000), lat, lng, alt, altFT_BMP, tempF_BMP);
        haveSerialData = true;
      } else {
        haveSerialData = false;
      }
    }  
    // other than runMode 2      
    if (haveSerialData) { 
#if 1      
        if (toggle) {
          digitalWrite(LED, HIGH);            
        } else {
          digitalWrite(LED, LOW);              
        }
        toggle = !toggle;
#endif       

        // Send a message!
        if (transActive == 1) {
          if (len > (RH_RF95_MAX_MESSAGE_LEN - 1)) len = (RH_RF95_MAX_MESSAGE_LEN - 1); // around 250
          // this odd-looking code is poor man's double buffering :)
          if (waiting == 1) {
            rf95.waitPacketSent(); 
          }
          waiting = 0;          
          memcpy(sendpacket,radiopacket,len);
          rf95.send((uint8_t *)sendpacket, len);
          waiting = 1;
          Serial.print("Transmitting: "); Serial.println(radiopacket);  
          // rf69.waitPacketSent();
        }
     }
}
/* end of code */
