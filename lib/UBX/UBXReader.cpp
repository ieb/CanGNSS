
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

void UBXReader::end() {
    ubloxDevice.end();
    // disable the pins by making them input pins.
    pinMode(PIN_PB2, INPUT);
    pinMode(PIN_PB3, INPUT);
    enabled = false;
}

bool UBXReader::begin(uint16_t baudRate) {
    console->println(F("ublox must be pre configured and running 19200 8-N-1"));

    ubloxDevice.begin(19200);
    console->print(F("Trying "));
    console->print(19200);
    console->print(F(" baud with rx buffer "));
    console->println(SERIAL_RX_BUFFER_SIZE);
    if ( detectTraffic() ) {
        console->println(F("Ok"));
        enabled = true;
        return true;
    } else {
        console->println(F("not 19200, press F to reset"));
        return false;
    }
}

void UBXReader::printResult(bool r) {
    if ( r ) {
        console->println(F(" Ok"));
    } else {
        console->println(F(" Fail"));
    }
}

void UBXReader::factoryReset() {
    if ( !enabled ) {

        console->println(F("Not enabled"));
        return;
    }
    // autobaud until we find the current baudrate
    if (autoBaud() ) {
        read(); // clear messages that were pending
        switchBaudRate(19200);
        console->println(F("Using baud 19200"));
        for (uint8_t i = 0; i < 16;i++) {
            // disable all nmea0183 messages, lookuo in uCenter
            console->print(F("Disable NMEA0183:"));
            console->print(i);
            printResult(setMessageRate(0xF0,i,0));
        }
        console->print(F("Satellite setup "));
        printResult(setupSatelites());
        console->print(F("PosLLH 5Hz "));
        printResult(setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_POSLLH, 1)); // 5Hz
        console->print(F("Velned 2.5Hz "));
        printResult(setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_VELNED, 2)); // 2.5Hz
        console->print(F("PVT 1Hz "));
        printResult(setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_PVT, 5)); // 1Hz
        console->print(F("DOP 1Hz "));
        printResult(setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_DOP, 6)); // 1Hz
        console->print(F("Sat 1Hz "));
        printResult(setMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_SAT, 9)); // 0.5Hz
        console->print(F("NavRate 200ms "));
        printResult(setNavRate(200,1,0));
        console->println(F("Reset Done"));
    } else {
        console->println(F("No baud found, reset failed"));
    }
}

bool UBXReader::autoBaud() {
    uint32_t bauds[] = {
        9600, 19200, 38400, 57600, 115200
    };
    if ( detectTraffic() ) {
        console->println(F("Baud Ok"));
        return true;
    } else {
        for (int i = 0; i < 5; i++) {
            switchBaudRate(bauds[i]);
            if ( detectTraffic() ) {
                console->print(F("Detected baud"));
                console->println(bauds[i]);
                return true;
            }
        }
        return false;
    }
}

bool UBXReader::detectTraffic() {
    // try to read 2 lines to see if we are getting NMEA0183
    unsigned long start = millis();
    uint8_t buf[4];
    while(millis()-start < 1000) {
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

void UBXReader::switchBaudRate(uint32_t toBaud) {
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
            (uint32_t) toBaud,
            (uint16_t) 3, // 3  ubx + nemea
            (uint16_t) 3, // 3  ubx + nemea
            (uint16_t) 0x0000, // 0 , no extended tx buffer.
            0xff, // reserved
            0xff, // reserved
            0x01,
            0x02
        };
    calculateChecksum(&(cfgUart.header), &(cfgUart.checksum[0]));
    //dumpMessage(&(cfgUart.header));
    ubloxDevice.write((uint8_t *)&cfgUart, sizeof(cfgUart));
    ubloxDevice.flush();
    delay(1000);
    ubloxDevice.end();
    delay(1000);
    
    ubloxDevice.begin(toBaud);
}

bool UBXReader::setMessageRate(uint8_t cls, uint8_t id, uint8_t rate) {
/*
    {
        uint8_t msg[] = {
            ULBOX_SYNC1,
            ULBOX_SYNC2,
            MSG_CLASS_CFG,
            MSG_ID_CFG_MSG,
            (uint8_t) 2,
            (uint8_t) 0,
            (uint8_t) cls,
            (uint8_t) id,
            (uint8_t) 0x01,
            (uint8_t) 0x02
        };
        calculateChecksum((UbloxHeader *)&(msg[0]), &(msg[8]));
        console->print("\nPoll>");
        dumpMessage((UbloxHeader *)&msg[0]);
        ubloxDevice.write((uint8_t *)&msg[0], 10);
        ubloxDevice.flush();
        for (int r = 0; r < 5; r++) {
            if (expect(MSG_CLASS_CFG, MSG_ID_CFG_MSG, false) ) {
                CfgMsg * resp = (CfgMsg *)ubloxHeader;
                if ( ubloxHeader->payloadLength == 8 && 
                    resp->messageClass == cls &&
                    resp->messageId == id) {
                    console->print(F("Rates:"));
                    for (int i = 0; i < 6; i++) {
                        console->print(resp->rates[i]);
                        console->print(",");
                    }
                    console->println("");
                    break;
                }
            }
        }
    } */
    {
        uint8_t msg[] = {
            ULBOX_SYNC1,
            ULBOX_SYNC2,
            MSG_CLASS_CFG,
            MSG_ID_CFG_MSG,
            (uint8_t) 3,
            (uint8_t) 0,
            (uint8_t) cls,
            (uint8_t) id,
            (uint8_t) rate,
            (uint8_t) 0x01,
            (uint8_t) 0x02,
        };
        return sendConfig((uint8_t *)&msg[0], 3+8);
    }
}

bool UBXReader::setNavRate(uint16_t measMs, uint16_t measCycles, uint16_t ref) {
    uint8_t msg[] = {
        ULBOX_SYNC1,
        ULBOX_SYNC2,
        MSG_CLASS_CFG,
        MSG_ID_CFG_RATE,
        (uint8_t) 6,
        (uint8_t) 0,
        (uint8_t) measMs&0xff,
        (uint8_t) (measMs>>8)&0xff,
        (uint8_t) measCycles&0xff,
        (uint8_t) (measCycles>>8)&0xff,
        (uint8_t) ref&0xff,
        (uint8_t) (ref>>8)&0xff,
        0x01,
        0x02
    };
    return sendConfig((uint8_t *)&msg[0], 6+8, false);
}

bool UBXReader::setupSatelites() {
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
    return sendConfig((uint8_t *)&message, sizeof(message));
}

void UBXReader::saveConfig() {
    if ( !enabled ) {
        return;
    }
    uint8_t msg[] = {
        ULBOX_SYNC1,
        ULBOX_SYNC2,
        MSG_CLASS_CFG,
        MSG_ID_CFG_CFG,
        (uint8_t) 13,
        (uint8_t) 0,
        (uint8_t) 0x00, // dont clear anything
        (uint8_t) 0x00, // dont clear anything
        (uint8_t) 0x00, // dont clear anything
        (uint8_t) 0x00, // dont clear anything
        (uint8_t) 0b00011011, // save port, msg, nav, gnss
        (uint8_t) 0x00, // save
        (uint8_t) 0x00, // save
        (uint8_t) 0x00, // save
        (uint8_t) 0x00, // load
        (uint8_t) 0x00, // load
        (uint8_t) 0x00, // load
        (uint8_t) 0x00, // load
        (uint8_t) 0x03, // device, BBR and Flash
        0x01,
        0x02
    };
    if ( sendConfig((uint8_t *)&msg[0], 13+8, false) ) {
        console->println(F("Config saved"));
    } else {
        console->println(F("Config save failed"));
    }
}

bool UBXReader::sendConfig(uint8_t * msg, uint16_t len, bool verbose) {
    UbloxHeader * h = (UbloxHeader *) msg;
    calculateChecksum(h, &(msg[len-2]));
    if ( verbose) {
        console->print("\n>");
        dumpMessage(h);        
    }
    ubloxDevice.write(msg,len);
    ubloxDevice.flush();
    return expectAck(h->messageClass, h->messageId, verbose);
}


bool UBXReader::expectAck(uint8_t clss, uint8_t id, bool verbose) {
    for (int r = 0; r < 1; r++) {
        if (expect(MSG_CLASS_ACK, 0xff, verbose) ) {
            CfgAck * resp = (CfgAck *)ubloxHeader;
            if ( ubloxHeader->payloadLength == 2 && 
                resp->messageClass == clss &&
                resp->messageId == id) {
                return ( ubloxHeader->messageId == MSG_ID_ACK_ACK);
            }
        }
    }
    return false;
}

bool UBXReader::expect(uint8_t clss, uint8_t id, bool verbose) {
    unsigned long start = millis();
    while(millis()-start < 2000) {
        eUbloxMessageStatus status = read();
        if (status == msgStatusOk || status == msgAck || status == msgNak) {
            if (verbose) {
                console->print("<");
                dumpMessage(ubloxHeader, false);
            }
            if ( ubloxHeader->messageClass == clss  && (id = 0xff || ubloxHeader->messageId == id)) {
                return true;
            }
        }
    }
    return false;
}


/**
 * read messages and call back with any message detected from the UbxMessageTypes
 * Configured.
 */
eUbloxMessageStatus UBXReader::read() {
    if ( !enabled ) {
        return msgIncomplete;
    }
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

void UBXReader::dumpMessage(UbloxHeader *message, bool withPayload) {
    console->print(F("Cls:0x"));
    console->print(message->messageClass,HEX);
    console->print(F(" Id:0x"));
    console->print(message->messageId,HEX);
    console->print(F(" len:"));
    console->print(message->payloadLength);
    console->print(F(" payload:"));
    if ( withPayload ) {
        uint8_t * buf = (uint8_t *)message; 
        dumpBufferHex(&buf[6],message->payloadLength);        
    }
    console->println("");
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

