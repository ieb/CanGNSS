
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SmallNMEA2000.h"
#include "commandline.h"
#include "GNSSReciever.h"
#include "UBXReader.h"
#include "SoftwareSerial.h"

/*
Reads messages from a UBLOX receiver and on receipt translates into a 
NMEA2000 Message. Does not store any messages and requires the UBLOX
receiver to have been pre-configured with the required messages.
Additional messages read will be dropped. Messages that are too long
will be skipped.
*/


#ifdef MEGATINYCORE_MCU            
#define SNMEA_SPI_CS_PIN PIN_PA4
#define SOFT_RX_PIN PIN_PB0
#define SOFT_TX_PIN PIN_PB1
#else
#define SNMEA_SPI_CS_PIN 4
#define SOFT_RX_PIN 2
#define SOFT_TX_PIN 3
#endif

SoftwareSerial console(SOFT_RX_PIN, SOFT_TX_PIN);


const SNMEA2000ProductInfo productInfomation PROGMEM={
                                       1300,                        // N2kVersion
                                       45,                         // Manufacturer's product code
                                       "GNSS",    // Manufacturer's Model ID
                                       "Luna GNSS",     // Manufacturer's Software version code
                                       "5.6.7.8 (2017-06-11)",      // Manufacturer's Model version
                                       "0000002",                  // Manufacturer's Model serial code
                                       0,                           // SertificationLevel
                                       1                            // LoadEquivalency
};
const SNMEA2000ConfigInfo configInfo PROGMEM={
      "GNSS Receiver",
      "Luna Technical Area",
      "https://github.com/ieb/CanGNSS"
};

uint8_t pgnEnableRegister = 0xff;


const unsigned long txPGN[] PROGMEM = { 
    126992L, // System Time, 1Hz
    129025L, // Position Rapid update 5Hz
    129026L, // COG/SOG Rapid Update 4Hz
    129029L, // Position data 1Hz
    129539L, // GNSS DOPs 1Hz
    129540L, // GNSS Satellites in View
    127258L, // Magnetic Variation
    SNMEA200_DEFAULT_TX_PGN
};





const unsigned long rxPGN[] PROGMEM = { 
  SNMEA200_DEFAULT_RX_PGN
};

const SNMEA2000DeviceInfo devInfo = SNMEA2000DeviceInfo(
  02,   // device serial number
  145,  // GNSS
  60 // Navigation
);

#define DEVICE_ADDRESS 30
#define DEVICE_SERIAL_NUMBER 45

GNSSReciever gnssReciever = GNSSReciever(DEVICE_ADDRESS, 
          &devInfo, 
          &productInfomation, 
          &configInfo, 
          &txPGN[0], 
          &rxPGN[0],
          SNMEA_SPI_CS_PIN,
          &console
);


CommandLine commandLine = CommandLine(&console, &gnssReciever);

#define MESSAGE_BUFFER_SIZE 512
uint8_t messageBuffer[MESSAGE_BUFFER_SIZE];
UBXReader ubxReader(&console, &messageBuffer[0], MESSAGE_BUFFER_SIZE);






bool diagnostics = false;
void setDiagnostics(bool enabled) {
  gnssReciever.setDiagnostics(enabled);
  diagnostics = enabled;
}



void setup() {
  console.begin(115200);
  console.println(F("GNSS Receiver start"));
  commandLine.begin();

  // If PA7 is pulled high, then the 
  // device starts up emitting serial messages on RX1/TX1
  // pin X low then the device starts with the Can device using SPI.
  gnssReciever.setSerialNumber(DEVICE_SERIAL_NUMBER);
  gnssReciever.setDeviceAddress(DEVICE_ADDRESS);
  gnssReciever.open();
  /*
  while ( !gnssReciever.open() ) {
    console.print(F("Failed to start CAN"));
    delay(5000);
  }
  */
  ubxReader.begin();
  console.println(F("Running..."));
}

void dumpMessage(UbloxHeader *message) {
  static uint16_t lastReadCalls = 0;
  static uint16_t lastBytesRead = 0;
  console.print(F("cls: 0x"));
  console.print(message->messageClass, HEX);
  console.print(F(" id: 0x"));
  console.print(message->messageId, HEX);
  console.print(F(" len: "));
  console.print(message->payloadLength);  
  console.print(F(" state:"));
  console.print(Serial.getStatus(),HEX);
  console.print(F(" reads:"));
  UBXReaderMetrics * metrics = ubxReader.getMetrics();
  uint16_t reads = metrics->readCalls-lastReadCalls;
  lastReadCalls = metrics->readCalls;
  console.print(reads);
  console.print(F(" readsize:"));
  if ( reads > 0) {
    uint16_t readSize = (metrics->bytesRead-lastBytesRead)/reads;
    console.println(readSize);
  } else {
    console.println(F("n/a"));
  }
  lastBytesRead = metrics->bytesRead;
}
void dumpMetrics() {
  static unsigned long tnext = 0;
  unsigned long now = millis();
  if ( now > tnext ) {
    UBXReaderMetrics * metrics = ubxReader.getMetrics();
    tnext = now + 5000;
    console.print(F("metrics bread:"));
    console.print(metrics->bytesRead);
    console.print(F(" restarts: "));
    console.print(metrics->bufferRestarts);
    console.print(F(" received: "));
    console.print(metrics->messageRecieved);
    console.print(F(" errors: "));
    console.print(metrics->messageError);
    console.print(F(" overflow: "));
    console.println(metrics->messageOverflow);
  }

}

void dumpFix() {
  static unsigned long tnext = 0;
  unsigned long now = millis();
  if ( now > tnext ) {
    GNSSFix  * fix = gnssReciever.getFix();

    tnext = now + 2000;
    console.print(F("fix lat:"));
    console.print(fix->lat);
    console.print(F(" lon:"));
    console.print(fix->lon);
    console.print(F(" cog:"));
    console.print(fix->heading_scaled);
    console.print(F(" sog:"));
    console.print(fix->ground_speed);
    console.print(F(" sats:"));
    console.print(fix->numSvu);
    console.print(F(" pdop:"));
    console.println(fix->pdop);
  }
}





void loop() {
  static uint16_t calls = 0;
  calls++;
  eUbloxMessageStatus status = ubxReader.read();
  UbloxHeader * message = ubxReader.getMessage();
  switch(status) {
    case msgStatusOk:
      if ( diagnostics || calls < 5) {
        console.print(calls);
        console.print(F(" Ok"));
        dumpMessage(message);      
      }
      gnssReciever.update(message);
      calls = 0;
      break;
    case msgStatusBadCheckSum:
      console.print(calls);
      console.println(F(" Bad checksum "));
      dumpMessage(message);
      calls = 0;
      break;
    case msgStatusOverflow:
      console.print(calls);
      console.println(F(" Overflow "));
      //dumpMessage(message);
      calls = 0;
      break;
    case msgAck:
      console.print(calls);
      console.println(F(" ACK"));
      calls = 0;
      break;
    case msgNak:
      console.print(calls);
      console.println(F(" NAK"));
      calls = 0;
      break;
    case msgIncomplete:
//      console.print(".");
      break;
  }
  gnssReciever.processMessages();
  commandLine.checkCommand();
  dumpFix();
  dumpMetrics();
}


