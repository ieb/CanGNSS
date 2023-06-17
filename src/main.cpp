
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SmallNMEA2000.h"
#include "commandline.h"
#include "UBXRead.h"
#include "SoftwareSerial.h"

/*
Reads messages from a UBLOX receiver and on receipt translates into a 
NMEA2000 Message. Does not store any messages and requires the UBLOX
receiver to have been pre-configured with the required messages.
Additional messages read will be dropped. Messages that are too long
will be skipped.
*/



#define SNMEA_SPI_CS_PIN PIN_PA4
#define CONSOLE_RX_PIN PIN_PB0
#define CONSOLE_TX_PIN PIN_PB1


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

SNMEA2000Device nmea2000Device = SNMEA2000Device(DEVICE_ADDRESS, 
          &devInfo, 
          &productInfomation, 
          &configInfo, 
          &txPGN[0], 
          &rxPGN[0],
          SNMEA_SPI_CS_PIN
);


#define MESSAGE_BUFFER_SIZE 128
const uint8_t messageBuffer[MESSAGE_BUFFER_SIZE];
UBXReader ubxReader(&Serial, &messageBuffer[0], MESSAGE_BUFFER_SIZE);

SofwareSerial console(CONSOLE_RX_PIN, CONSOLE_TX_PIN);

CommandLine commandLine = CommandLine(&console, &pressureMonitor);


bool diagnostics = false;
void setDiagnostics(bool enabled) {
  pressureMonitor.setDiagnostics(enabled);
  diagnostics = enabled;
}


void setup() {
  console.begin(9600);
  Serial.begin(115200);
  console.println(F("GNSS Receiver start"));
  commandLine.begin();

  // If PA7 is pulled high, then the 
  // device starts up emitting serial messages on RX1/TX1
  // pin X low then the device starts with the Can device using SPI.
  nmea2000Device.setSerialNumber(DEVICE_SERIAL_NUMBER);
  nmea2000Device.setDeviceAddress(DEVICE_ADDRESS);
  while ( !nmea2000Device.open() ) {
    console.print(F("Failed to start CAN"));
    delay(5000);
    digitalWrite(FAILURE_LED, HIGH);
  }
  console.println(F("Running..."));
}

void processMessage() {
  UbloxHeader * message = ubxReader.getMessage();
  switch(message->messageClass) {
    case 0x01:
      switch(message->messageId) {
        case 0x02:
      }
    break;
    default:
    break;
  }
  switch (messageTypeIdx) {
    case 0:
       NavPosLLH * pos = (NavPosLLH *)(&packetBuffer);
       break;
    case 1:
      NavStatus * status = (NavStatus *)(&packetBuffer);
      break;
  }
}

void loop() {
  if ( ubxReader.read() == msgStatusOk ) {
    processMessage();
  }
  nmea2000Device.processMessages();
  commandLine.checkCommand();
}


