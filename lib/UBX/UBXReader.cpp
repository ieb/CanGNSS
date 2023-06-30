
#include "UBXReader.h"

#define ubloxDevice Serial



/**
 * Call the UBX reader with a list of UbxMessageTypes
 */
UBXReader::UBXReader(Stream * console, uint8_t * buffer, uint16_t len) {
    this->console = console;
    packetBuffer = buffer;
    ubloxHeader = (UbloxHeader *) buffer;
    maxBufferSize = len;
};

//#define CONFIGURE_DEVICE 1

bool UBXReader::begin(uint16_t baudRate) {
#ifndef CONFIGURE_DEVICE
    console->println(F("ublox must be pre configured and running 19200 8-N-1"));
    ubloxDevice.begin(baudRate);
    console->print(F("Trying 19200 baud with rx buffer "));
    console->print(SERIAL_RX_BUFFER_SIZE);
    if ( detectTraffic() ) {
        return true;
    } else {
        return false;
    }
#else
    return configureDevice(baudRate);
#endif
}

bool UBXReader::detectTraffic() {
    // try to read 2 lines to see if we are getting NMEA0183
    unsigned long timeout = millis()+1000;
    uint8_t buf[4];
    while(millis() < timeout) {
        if ( ubloxDevice.available() ) {
            buf[0] = buf[1];
            buf[1] = buf[2];
            buf[2] = buf[3];
            buf[3] = ubloxDevice.read();
//                console->write(buf[3]);
            if ( buf[0] == '\r' && buf[1] == '\n' && buf[2] == '$' && buf[3] == 'G') {
                //NMEA0183 detected.
                console->println(F(" NMEA0183 detected"));
                return true;
            } else if ( buf[0] == ULBOX_SYNC1 && buf[1] == ULBOX_SYNC2 ) {
                // UBX detected.
                console->println(F(" UBX detected"));
                return true;
            }
        }
    }
    console->println(F(" Nothing detected"));
    return false;
}

#ifdef CONFIGURE_DEVICE


// Only required were the device is not pre configured.
// removed to save progmem space

bool UBXReader::configureDevice(uint16_t baudRate) {
    // discover what state the device is in, and setup
    // to match what is required.
    bool setupDone = false;
    uint8_t baudId = 0;
    uint32_t bauds[] = { 38400, 57600, 115200,  9600,  4800, 19200};
    uint8_t nbauds = 6;

    while(!setupDone) {
        baudId = 0;
        bool detecting = true;
        for (; baudId < nbauds; baudId++) {
            ubloxDevice.begin(bauds[baudId]);
            console->print(F("Trying "));
            console->print(bauds[baudId]);

            if ( detectTraffic() ) {
                detecting = false;
            }

            if ( detecting ) {
                ubloxDevice.end();
                console->println(F(" disconnected"));
            } else {
                break;
            }
        }
        if (detecting) {
            console->println(F(" Ublox device not detected at any baud"));
            return false;
        }
        if ( bauds[baudId] != baudRate ) {
            // reconfigure to baudID 0, and restart.
            CfgUart cfgUart = {
                ULBOX_SYNC1,
                ULBOX_SYNC2,
                MSG_CLASS_CFG, 
                MSG_ID_CFG_PRT, 
                (uint16_t) 20, 
                0x01, // UART1 
                0xff, // reserved
                (uint16_t) 0x00, // txReady disabled.
                (uint32_t) 0x000008C0, // 8-N-1
                (uint32_t) baudRate,
                (uint16_t) 3, // 3  ubx + nemea
                (uint16_t) 3, // 3  ubx + nemea
                (uint16_t) 0x0000, // 0 , no extended tx buffer.
                0xff, // reserved
                0xff, // reserved
                0x01,
                0x02
            };
#ifdef DEBUG
            console->print(F("before checksum:        [ "));
            dumpBufferHex((uint8_t *)&cfgUart, sizeof(cfgUart));
            console->println(F("]"));
#endif
            calculateChecksum(&(cfgUart.header), &(cfgUart.checksum[0]));
#ifdef DEBUG
            console->print(F("Sent baudrate message as:[ "));
            dumpBufferHex((uint8_t *)&cfgUart, sizeof(cfgUart));
            console->println(F("]"));
#endif
            ubloxDevice.write((uint8_t *)&cfgUart, sizeof(cfgUart));
            ubloxDevice.flush();
            delay(100);
            ubloxDevice.end();
        } else {
            setupDone = true;
        }

    }


    // setup the navigation messages with MSG, disabling all NMEA0183 messages
    // and enabling the ones we want
    // set up the navigation rates with rate 200ms nav period 1 nav cycle per event.
    //
    for (uint8_t i = 0; i < 16;i++) {
        setMessageRate(0xF0,i,0); // disable all nmea0183 messages, lookuo in uCenter
    }
    setupSatelites();
    setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_POSLLH, 1); // 5Hz
    setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_VELNED, 2); // 2.5Hz
    setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_PVT, 5); // 1Hz
    setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_DOP, 5); // 1Hz
    setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_SAT, 10); // 0.5Hz
    setNavRate(200,1,0);

    return true;

}

void UBXReader::setMessageRate(uint8_t cls, uint8_t id, uint8_t rate) {
    CfgMsg messageRate = {
        ULBOX_SYNC1,
        ULBOX_SYNC2,
        MSG_CLASS_CFG,
        MSG_ID_CFG_MSG,
        (uint16_t) 8,
        cls, // Message class
        id, // Message ID
        0x00, //i2c
        rate, // 1Hz on UART1
        0x00, // Uart2
        0x00, //USB
        0x00, // SPI
        0x00,  // ??
        0x01, // CHK A
        0x02 // CHK B
    };
    calculateChecksum(&(messageRate.header), &(messageRate.checksum[0]));
    ubloxDevice.write((uint8_t *)&messageRate, sizeof(CfgMsg));
    ubloxDevice.flush();
    delay(50);

}
void UBXReader::setNavRate(uint16_t measMs, uint16_t measCycles, uint16_t ref) {
    CfgRate rates = {
        ULBOX_SYNC1,
        ULBOX_SYNC2,
        MSG_CLASS_CFG,
        MSG_ID_CFG_MSG,
        (uint16_t) 6,
        measMs,
        measCycles,
        ref,
        0x01,
        0x02
    };
    calculateChecksum(&(rates.header), &(rates.checksum[0]));
    ubloxDevice.write((uint8_t *)&rates, sizeof(CfgRate));
    ubloxDevice.flush();
    delay(50);
}
void UBXReader::setupSatelites() {
    CfgGNSS message;

    message.header.sync1 = ULBOX_SYNC1;
    message.header.sync2 = ULBOX_SYNC2;
    message.header.messageClass = MSG_CLASS_CFG;
    message.header.messageId = MSG_ID_CFG_GNSS;
    message.header.payloadLength = (4+7*sizeof(CfgGNSS_System));
    message.msgVer = 0;
    message.numTrkChHw = 32;
    message.numTrkChUse = 0xFF;
    message.numConfigBlocks = 7;

    message.systems[0].gnssId = 0; // GPS 
    message.systems[0].resTrkCh = 8; 
    message.systems[0].maxTrkCh = 16; 
    message.systems[0].flags = 0x00010001; // L1C/A 1575.42MHz 

    message.systems[1].gnssId = 1; // SBAS
    message.systems[1].resTrkCh = 1; 
    message.systems[1].maxTrkCh = 3; 
    message.systems[1].flags = 0x00010001; // L1C/A 1575.42MHz 

    message.systems[2].gnssId = 2; // Galleleo
    message.systems[2].resTrkCh = 4; 
    message.systems[2].maxTrkCh = 10; 
    message.systems[2].flags = 0x00010001; // E1 1,575.42MHz

    message.systems[3].gnssId = 3; // BeiDou, disabled
    message.systems[3].resTrkCh = 0; 
    message.systems[3].maxTrkCh = 0; 
    message.systems[3].flags = 0x00000000; 

    message.systems[4].gnssId = 4; // IMES, disabled
    message.systems[4].resTrkCh = 0; 
    message.systems[4].maxTrkCh = 0; 
    message.systems[4].flags = 0x00000000; 

    message.systems[5].gnssId = 5; // QZSS, disabled
    message.systems[5].resTrkCh = 0; 
    message.systems[5].maxTrkCh = 0; 
    message.systems[5].flags = 0x00000000; 

    message.systems[6].gnssId = 5; // Glonas 
    message.systems[6].resTrkCh = 6; 
    message.systems[6].maxTrkCh = 8; 
    message.systems[6].flags = 0x00010001;  // L1  1592.9525 MHz to 1610.485 14 channels

    // so we can verify checksump calculation.
    message.checksum[0] = 0x01;
    message.checksum[1] = 0x02;

    calculateChecksum(&(message.header), &(message.checksum[0]));
    ubloxDevice.write((uint8_t *)&message, sizeof(CfgGNSS));
    ubloxDevice.flush();
    delay(50);

}

#endif

/**
 * read messages and call back with any message detected from the UbxMessageTypes
 * Configured.
 */
eUbloxMessageStatus UBXReader::read() {
    metrics.readCalls++;
    while (ubloxDevice.available() > 0) {
        uint8_t c = ubloxDevice.read();
#ifdef DEBUG
        if ((c > 31 && c < 127) || c == '\r' || c == '\n') {
            console->write(c);
        } else {
            console->print("[0x");
            console->print(c,HEX);
            console->print("]");
        }
#endif
        if ( pos < maxBufferSize ) {
            packetBuffer[pos] = c;
        }
        metrics.bytesRead++;
        pos++;
        if ( pos == 1 ) {
            if ( c != ULBOX_SYNC1 ){
                metrics.bufferRestarts++;
                pos = 0;                
            } 
        } else if (pos == 2) {
            if ( c != ULBOX_SYNC2 ){
                metrics.bufferRestarts++;
                pos = 0;                
            } 
        } else if ( pos >= sizeof(UbloxHeader) ) {
            // not supporting messages larger than 2K, so 
            // assume thats a corruption and scan for the next message
            if ( ubloxHeader->payloadLength > 2048 ) {
                    metrics.messageOverflow++;
                    pos = 0;
                    return msgStatusOverflow;                
            }
            if ( pos == (sizeof(UbloxHeader) + ubloxHeader->payloadLength + 2) ) {
                metrics.messageRecieved++;
                if ( pos < maxBufferSize ) {
                    if ( isChecksumCorrect() ) {
                        pos = 0;
                        if ( ubloxHeader->messageClass == MSG_CLASS_ACK ) {
                            if (ubloxHeader->messageId == MSG_ID_ACK_ACK) {
                                return msgAck;
                            } else if (ubloxHeader->messageId == MSG_ID_ACK_NAK ) {
                                return msgNak;
                            }
                        }
                        return msgStatusOk;
                    } else {
                        metrics.messageError++;
                        pos = 0;
                        return msgStatusBadCheckSum;
                    }
                } else {
                    metrics.messageOverflow++;
                    pos = 0;
                    return msgStatusOverflow;
                }
            }
        }
    }
    return msgIncomplete;
}


void UBXReader::dumpBufferHex(uint8_t * buffer, uint16_t len) {
    for(uint16_t i = 0; i < len; i++) {
        if (buffer[i] < 16) {
            console->print(F(" 0x0"));
        } else {
            console->print(F(" 0x"));
        }
        console->print(buffer[i],HEX);
        if (i%20 == 19) {
            console->println("");
        }
    }
}

UbloxHeader *  UBXReader::getMessage() {
    return ubloxHeader;
}

bool UBXReader::isChecksumCorrect() {
  uint8_t b[] = {0,0};
  calculateChecksum(ubloxHeader, &b[0]);
  uint16_t messageLength =  ubloxHeader->payloadLength + sizeof(UbloxHeader);
  if (b[0] == packetBuffer[messageLength] && b[1] == packetBuffer[messageLength+1]) {
    return true;
  } else {
    // bad checksum.
    console->print(F("Bad Checksum payloadlen:"));
    console->print(ubloxHeader->payloadLength);
    console->print(F(" chkidx:"));
    console->print(messageLength);
    console->print(F(" state:"));
    console->print(ubloxDevice.getStatus(),HEX);
    console->print(F(" pos:"));

    console->println(pos);
    console->print(F("0x"));
    console->print(b[0],HEX);
    console->print(F(" 0x"));
    console->println(b[1],HEX);
    console->print(F("0x"));
    console->print(packetBuffer[messageLength],HEX);
    console->print(F(" 0x"));
    console->println(packetBuffer[messageLength+1],HEX);
    dumpBufferHex(packetBuffer, messageLength+2);
    console->println("");
    return false;
  }
}

void UBXReader::calculateChecksum(UbloxHeader *message, uint8_t *op) {
  uint8_t a = 0, b = 0;
  uint8_t *messageBuffer = (uint8_t *)message;
  uint16_t messageLength =  message->payloadLength + sizeof(UbloxHeader);
  // dont checksum the sync bytes or the checksum byte itself.
  for (uint16_t i = 2; i < messageLength; i++) {
    a =  a + messageBuffer[i];
    b =  b + a;
#ifdef DEBUG
    console->print("idx:");
    console->print(i);
    console->print(" in:0x");
    console->print(messageBuffer[i], HEX);
    console->print(" a:");
    console->print(a);
    console->print("b:");
    console->println(b);
#endif
  }
  op[0] = a;
  op[1] = b;    
}

