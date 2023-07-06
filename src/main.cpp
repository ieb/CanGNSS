
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
Print *consolePtr = &console;


const SNMEA2000ProductInfo productInfomation PROGMEM={
                                       1300,                        // N2kVersion
                                       46,                         // Manufacturer's product code
                                       "5Hz GNSS",    // Manufacturer's Model ID
                                       "0817f57",     // Manufacturer's Software version code
                                       "5Hz GNSS",      // Manufacturer's Model version
                                       "0000002",                  // Manufacturer's Model serial code
                                       0,                           // SertificationLevel
                                       1                            // LoadEquivalency
};
const SNMEA2000ConfigInfo configInfo PROGMEM={
      "5Hz GNSS Receiver",
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


CommandLine commandLine = CommandLine(&console);

#define MESSAGE_BUFFER_SIZE 512
uint8_t messageBuffer[MESSAGE_BUFFER_SIZE];
UBXReader ubxReader(&console, &messageBuffer[0], MESSAGE_BUFFER_SIZE);


void factoryReset() {
  ubxReader.factoryReset();
}

void saveConfig() {
  ubxReader.saveConfig();
}

void disconnectUBX() {
  ubxReader.end();
  console.println(F("UBX disconnected, reboot to reconnect"));
}

uint8_t diagnostics = 0;
#define INFO_ENABLED  (diagnostics > 0)
#define DEBUG_ENABLED  (diagnostics > 1)

void changeDiagnostics() {
  diagnostics = (diagnostics+1)%3;
  if ( DEBUG_ENABLED ) {
    console.println(F("DEBUG ON"));
    gnssReciever.setDiagnostics(true);
  } else if (INFO_ENABLED) {
    console.println(F("INFO ON"));
    gnssReciever.setDiagnostics(false);
  } else {
    console.println(F("INFO OFF"));
    gnssReciever.setDiagnostics(false);
  }
}




void setup() {
  console.begin(115200);
  console.println(F("GNSS Receiver start"));
  commandLine.begin();

  // required to avoid an error initialising the can bus.
  pinMode(SNMEA_SPI_CS_PIN, OUTPUT);

  // If PA7 is pulled high, then the 
  // device starts up emitting serial messages on RX1/TX1
  // pin X low then the device starts with the Can device using SPI.
  gnssReciever.setSerialNumber(DEVICE_SERIAL_NUMBER);
  gnssReciever.setDeviceAddress(DEVICE_ADDRESS);
  gnssReciever.open();
  int retries = 0;
  while ( retries < 5 && !gnssReciever.open() ) {
    console.println(F("Failed to start CAN, retrying in 5s"));
    delay(5000);
    retries++;
  }
  if ( retries == 5 ) {
    console.println(F("Failed to start CAN, no CAN"));
  }
  ubxReader.begin(57600);
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

void dumpMetrics(bool force) {
  static unsigned long tnext = 0;
  unsigned long now = millis();
  if ( force || now > tnext ) {
    UBXReaderMetrics * metrics = ubxReader.getMetrics();
    GNSSMetrics * gnssMetrics = gnssReciever.getMetrics();
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
    console.print(metrics->messageOverflow);
    console.print(F(" posllh: "));
    console.print(gnssMetrics->posllh);
    console.print(F(" velned: "));
    console.print(gnssMetrics->velned);
    console.print(F(" pvt: "));
    console.print(gnssMetrics->pvt);
    console.print(F(" dop: "));
    console.print(gnssMetrics->dop);
    console.print(F(" sat: "));
    console.print(gnssMetrics->sat);
    console.print(F(" unknown: "));
    console.println(gnssMetrics->unknown);
  }

}

/*
typedef struct _GNSSFix {
    uint8_t sid;
    uint8_t fixType;
    uint8_t methodType;
    uint8_t actualMode;
    uint8_t valid;
    uint8_t numSV;
    uint8_t numSvu;
    int16_t pdop;
    int16_t hdop;
    int16_t vdop;
    int16_t tdop;
    uint16_t daysSince1970;
    uint32_t secondsSinceMidnight;
    int32_t lat;
    int32_t lon;
    int32_t height;
    int32_t latitude_scaled;
    int32_t longitude_scaled;
    int32_t heading_scaled;
    uint32_t ground_speed;
    float variation;
} GNSSFix;
*/
void dumpFix(bool force) {
  static unsigned long tnext = 0;
  unsigned long now = millis();
  if ( force || now > tnext ) {
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
    console.print(fix->pdop);
    console.print(F(" daysSince1970:"));
    console.print(fix->daysSince1970);
    console.print(F(" height:"));
    console.println(fix->height);
  }
}



void dumpStatus() {
  gnssReciever.dumpStatus();
  dumpMetrics(true);
  dumpFix(true);
}


void loop() {
  static uint16_t calls = 0;
  calls++;
  eUbloxMessageStatus status = ubxReader.read();
  UbloxHeader * message = ubxReader.getMessage();
  switch(status) {
    case msgStatusOk:
      if ( DEBUG_ENABLED ) {
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
  if ( INFO_ENABLED ) {
    dumpMetrics(false);    
    dumpFix(false);
  }
}


