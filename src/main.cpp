
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

#define MESSAGE_BUFFER_SIZE 128
uint8_t messageBuffer[MESSAGE_BUFFER_SIZE];
UBXReader ubxReader(&console, &messageBuffer[0], MESSAGE_BUFFER_SIZE);






bool diagnostics = false;
void setDiagnostics(bool enabled) {
  gnssReciever.setDiagnostics(enabled);
  diagnostics = enabled;
}


void setup() {
  console.begin(34800);
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
  console.print(F("cls: 0x"));
  console.print(message->messageClass, HEX);
  console.print(F(" id: 0x"));
  console.print(message->messageId, HEX);
  console.print(F(" len: 0x"));
  console.println(message->payloadLength);  
}
void dumpMetrics() {
  static unsigned long tnext = 0;
  unsigned long now = millis();
  if ( now > tnext ) {
    tnext = now + 5000;
    UBXReaderMetrics * metrics = ubxReader.getMetrics();
    console.print(F("metrics bread:"));
    console.print(metrics->bytesRead);
    console.print(F(" metrics restarts: "));
    console.print(metrics->bufferRestarts);
    console.print(F(" metrics received: "));
    console.print(metrics->messageRecieved);
    console.print(F(" metrics errors: "));
    console.print(metrics->messageError);
    console.print(F(" metrics overflow: "));
    console.println(metrics->messageOverflow);
  }

}



void loop() {
  eUbloxMessageStatus status = ubxReader.read();
  UbloxHeader * message = ubxReader.getMessage();
  switch(status) {
    case msgStatusOk:
      console.print(F("Ok clsID:"));
      dumpMessage(message);
      gnssReciever.update(message);
      break;
    case msgStatusBadCheckSum:
      console.print(F("Bad checksum "));
      dumpMessage(message);
      break;
    case msgStatusOverflow:
      console.print(F("Overflow "));
      dumpMessage(message);
      break;
    case msgIncomplete:
      break;
  }
  dumpMetrics();
  gnssReciever.processMessages();
  commandLine.checkCommand();
}


