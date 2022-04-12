/**************************************************************************/
/*! 
    @file     iso14443a_uid.pde
    @author   Adafruit Industries
  @license  BSD (see license.txt)

    This example will attempt to connect to an ISO14443A
    card or tag and retrieve some basic information about it
    that can be used to determine what type of card it is.   
   
    Note that you need the baud rate to be 115200 because we need to print
  out the data and read from the card at the same time!

This is an example sketch for the Adafruit PN532 NFC/RFID breakout boards
This library works with the Adafruit NFC breakout 
  ----> https://www.adafruit.com/products/364
 
Check out the links above for our tutorials and wiring diagrams 
These chips use SPI or I2C to communicate.

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

*/
/**************************************************************************/

#define PN532DEBUG 
#define MIFAREDEBUG

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>


#define PN532_SS   (10)
#define PN532_MOSI (11)
#define PN532_MISO (12)
#define PN532_SCK  (13)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield
#define PN532_IRQ   (2)

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a SPI connection:
// Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
/*
20:33:33.896 -> Sending : 0x0, 0x0, 0xFF, 0x2, 0xFE, 0xD4, 0x2, 0x2A, 0x0, 
20:33:33.965 -> Sending : 0x0, 0x0, 0xFF, 0x2, 0xFE, 0xD4, 0x2, 0x2A, 0x0, 
20:33:33.965 -> Reading:  0x0 0x0 0xFF 0x6 0xFA 0xD5 0x3 0x32 0x1 0x6 0x7 0xE8
20:33:33.990 -> Found chip PN532
20:33:33.990 -> Firmware ver. 1.6
20:33:33.990 -> Sending : 0x0, 0x0, 0xFF, 0x5, 0xFB, 0xD4, 0x14, 0x1, 0x14, 0x1, 0x2, 0x0, 
20:33:34.085 -> TIMEOUT!
20:33:34.085 -> Waiting for an ISO14443A card
20:33:34.085 -> About to inList passive targetSending : 0x0, 0x0, 0xFF, 0x4, 0xFC, 0xD4, 0x4A, 0x1, 0x0, 0xE1, 0x0, 
20:33:35.160 -> TIMEOUT!
*/
/*

A = Arduino  P=PN523  C=Card

A1 with 5V
A1 - i2c - P1   --> fail 8050-Command
A1 - SPI - P1   --> Timeout on passive targetSending
  try 5V to P1    --> Timeout on passive targetSending
  try external 5V to P1    --> Timeout on passive targetSending
A1 - SPI - P1 + ext 5V + 2nd USB2Serial -> 
     ext 5V missed -> IRQ received + Reading:  0x0 0x0 0xFF 0x0 0xFF 0x0 + Timeout
     ext 5V - C1 -> 0xA4->0x9000 dann 0x80-> timeout
     ext 5V - C2 -> Responsecode 0x69FF
     disable PN532DEBUG -> Responsecode 0x69FF
A2 with 3V
A2 - SPI - P2   --> Timeout on passive targetSending
  try 3V to P2    --> Timeout on passive targetSending
  try 5V to P2    --> Timeout on passive targetSending
  disable PN532DEBUG -> Didn't find PN53x board
 */
// use Arduino Uno
// Adafruit_PN532 nfc(PN532_SS); - can find board
// Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS); // waiting for card ... 
// fixed connection-order
// change back to micro pro
// Adafruit_PN532 nfc(PN532_SS); --> dont find board
// Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET); // I2C
// change to FTDI-5 / NFC-3 / A-4 ... unable to upload scetch
// change to FTDI-5 / NFC-3 / A-6 ... unable to upload scetch
// A-Nano-7 / NFC-3
// A-Nano-7 / NFC-3  -- old bootloader !!!!
// Adafruit_PN532 nfc(PN532_SS); -- no board
// Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
// Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET); // --> 69FF
// #define PN532_SS   (8) // https://forum.arduino.cc/t/pn532-spi-connection/664737/4
// Adafruit_PN532 nfc(PN532_SS); // dont find board ...
//https://forum.arduino.cc/t/library-pn532-for-nano-33-ble/894185 -> Adafruit_PN532.cpp old code new Adafruit_SPIDevice(ss, clk, miso, mosi, 1000000, SPI_BITORDER_LSBFIRST, SPI_MODE0);} new code new Adafruit_SPIDevice(ss, clk, miso, mosi, 100000, SPI_BITORDER_LSBFIRST, SPI_MODE0);} …
// external 5V NFC // PIN8 PN532_SS -> Pin10
// 3,3V NFC
// Adafruit_PN532 nfc(PN532_SS);
// Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS); // --> Adafruit_PN532.cpp - PATCH -> 69FF
// 5V -> 69FF
// Patch new Adafruit_SPIDevice(ss, 100000, SPI_BITORDER_LSBFIRST, SPI_MODE0); tooo
// Adafruit_PN532 nfc(PN532_SS); // 69FF
// change to 3,3V -> no-board
// change to 5V -> no-board 
// change back to ext 5V -> no-board 
// --- long time usb deatched -----------
// Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS); // 69FF
// Adafruit_PN532 nfc(PN532_SS); // 69FF
// PN532_debug.h -> not used ...
// Adfruit_PN532.cpp - PN532DEBUG
// A-7 / N-8 -> Timeout
// A-7 / N-3 
//Adafruit_PN532 nfc(PN532_SS); --> Timeout
//Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS); --> timeout
// A-9 / N-2 -> no board
// 5V -> n.b.
// Adafruit_PN532 nfc(PN532_SS);
// N-8 ---> must set SPI
// FTDI-12 5V / A-13 / N-8  -> found board / timeout
// Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
// Adafruit_PN532 nfc(PN532_SS); // --> Timeout

// -- breakout board ..
// Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET); // i2c  2 = irq / 3 = reset -->  ok -> Select ok und dann Error 
// Adafruit_PN532 nfc(PN532_SS); //  board ok / waiting f. card
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);


/*
1:01:38.064 -> CARD FOUND
01:01:38.064 -> Sending : 0x0, 0x0, 0xFF, 0xE, 0xF2, 0xD4, 0x40, 0x1, 0x0, 0xA4, 0x4, 0x0, 0x5, 0x53, 0x65, 0x63, 0x75, 0x72, 0x0, 0x3C, 0x0, 
01:01:38.112 -> Reading:  0x0 0x0 0xFF 0x5 0xFB 0xD5 0x41 0x0 0x90 0x0 0x5A 0x0 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA
01:01:38.158 -> responseLength: 2
01:01:38.158 -> 90 00  ⸮.
01:01:38.158 -> /select |Secur
01:01:38.158 -> openSecureChannel... 
01:01:38.158 -> Sending : 0x0, 0x0, 0xFF, 0x11, 0xEF, 0xD4, 0x40, 0x1, 0x80, 0x50, 0x0, 0x0, 0x8, 0x1, 0x1, 0x2, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0, 0xA, 0x0, 
01:01:38.204 -> Reading:  0x0 0x0 0xFF 0x5 0xFB 0xD5 0x41 0x0 0x69 0xFF 0x82 0x0 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA 0xAA
01:01:38.251 -> responseLength: 2
01:01:38.251 -> 69 FF  i⸮
01:01:38.251 -> openSecureChannel ... FAIL 2 no ack

*/


// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
// Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
//Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);


void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10); // for Leonardo/Micro/Zero
  Serial.println("Hello!");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  //nfc.setPassiveActivationRetries(0xFF);
  //nfc.setPassiveActivationRetries(0xFE); // 0xFF = endless retry
  
  // configure board to read RFID tags
  nfc.SAMConfig();
  randomSeed(analogRead(0));
  
  Serial.println("Waiting for an ISO14443A card");
}

uint8_t rnd() {
  return random(0,256) & 0xff;
}

void loop(void) {
  boolean success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  // success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
  
  if (success) {
    Serial.println("Found a card!");
    Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("UID Value: ");
    for (uint8_t i=0; i < uidLength; i++) 
    {
      Serial.print(" 0x");Serial.print(uid[i], HEX); 
    }
    Serial.println("");
    // Wait 1 second before continuing
    delay(1000);
  }
    
  // set shield to inListPassiveTarget
  success = nfc.inListPassiveTarget();
 
  if(success) {
    // https://de.wikipedia.org/wiki/Application_Protocol_Data_Unit
    Serial.println("------------------------------------------------");
    Serial.println("CARD FOUND");
    uint8_t selectApdu[] = {0x00, /* CLA */  0xA4, /* INS */ 0x04, /* P1 ) */  0x00, /* P2  */
                            0x05, /* Lc  */  0x53, 0x65, 0x63, 0x75, 0x72, /* Data = Lc*/
                            0x00  /* Le */ };
    uint8_t response[255];
    memset(response, 0, sizeof(response));
    uint8_t responseLength = sizeof(response);  
     
    success = nfc.inDataExchange(selectApdu, sizeof(selectApdu), response, &responseLength);
    if(success) {
      Serial.print("responseLength: "); Serial.println(responseLength);
      nfc.PrintHexChar(response, responseLength);
      if (2 == responseLength && 0x90 == response[responseLength-2] && 0x00 == response[responseLength-1]) {
        Serial.println("/select |Secur");

        Serial.println("openSecureChannel... "); ; // INITIALIZE UPDATE
        uint8_t openSecureChannel[] = {0x80 /*CLA_GP*/,0x50 /*INIT_UPDATE*/,0x00/*Key-Version*/,0x00/*KeyId*/
              //, 0x08 /* Rand-Len */, 0xF1 /*Rand[8]*/,0x54,0xF8,0x92,0x22,0xA9,0x97,0x32
              //, 0x08 , rnd(), rnd(), rnd(), rnd(), rnd(), rnd(), rnd()
              , 0x08 /* Rand-Len */, 0x01, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01
              ,/*END*/0x00};
        responseLength = sizeof(response); // 0x6700 - wrong length !!
        uint8_t response2[10];
        memset(response2, 0, sizeof(response2));
        uint8_t responseLength2 = sizeof(response2);  

        if(success = nfc.inDataExchange(openSecureChannel, sizeof(openSecureChannel), response2, &responseLength2)) {
          Serial.print("responseLength: "); Serial.println(responseLength2);
          nfc.PrintHexChar(response2, responseLength2);
          if (0x9000 == (response2[responseLength2-2] << 8 | response2[responseLength2-1])) {
        
            Serial.println("openSecureChannel !!!!!");
            Serial.println("WELCOME NEXT STEP!!!!");
    
          } else {
            Serial.println("openSecureChannel ... FAIL 2 no ack"); ;
          }
        } else {
          Serial.println("openSecureChannel ... FAIL 1 com"); ;
          
        }

        
      } else {
        Serial.println("Keep trying!!");
      }
      Serial.println("------------------------------------------------");
    } 
    
    
  // Wait 1 second before continuing
  delay(20000);
  }
  else
  {
    // PN532 probably timed out waiting for a card
    Serial.println("Timed out waiting for a card");
    delay(5000);
  }
}
