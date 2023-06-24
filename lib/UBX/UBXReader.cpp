
#include "UBXReader.h"
#include <avr/wdt.h>


#define DESIRED_BAUD 38400

const uint32_t BAUDS[] = {
    38400, 57600, 115200,  9600,  4800, 19200
};
#define NUM_BAUDS 6

/**
 * Call the UBX reader with a list of UbxMessageTypes
 */
UBXReader::UBXReader(Stream * console, uint8_t * buffer, uint16_t len) {
    this->console = console;
    packetBuffer = buffer;
    ubloxHeader = (UbloxHeader *) buffer;
    maxBufferSize = len;
};


bool UBXReader::begin() {
    // discover what stat the device is in. 
    bool setupDone = false;
    uint8_t baudId = 0;
    while(!setupDone) {
        baudId = 0;
        bool detecting = true;
        for (; baudId < NUM_BAUDS; baudId++) {
            ubloxDevice.begin(BAUDS[baudId]);
            console->print(F("Trying "));
            console->print(BAUDS[baudId]);

            // try to read 2 lines to see if we are getting NMEA0183
            unsigned long timeout = millis()+1000;
            uint8_t buf[4];
            while(millis() < timeout && detecting) {
                if ( ubloxDevice.available() ) {
                    buf[0] = buf[1];
                    buf[1] = buf[2];
                    buf[2] = buf[3];
                    buf[3] = ubloxDevice.read();
    //                console->write(buf[3]);
                    if ( buf[0] == '\r' && buf[1] == '\n' && buf[2] == '$' && buf[3] == 'G') {
                        //NMEA0183 detected.
                        console->println(F(" NMEA0183 detected"));
                        detecting = false;
                    } else if ( buf[0] == ULBOX_SYNC1 && buf[1] == ULBOX_SYNC2 ) {
                        // UBX detected.
                        console->println(F(" UBX detected"));
                        detecting = false;
                    }
                }
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
        if ( BAUDS[baudId] != DESIRED_BAUD ) {
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
                (uint32_t) DESIRED_BAUD,
                (uint16_t) 3, // 3  ubx + nemea
                (uint16_t) 3, // 3  ubx + nemea
                (uint16_t) 0x0000, // 0 , no extended tx buffer.
                0xff, // reserved
                0xff, // reserved
                0x01,
                0x02
            };
            console->print(F("before checksum:        [ "));
            dumpBufferHex((uint8_t *)&cfgUart, sizeof(cfgUart));
            console->println(F("]"));
            calculateChecksum(&(cfgUart.header), &(cfgUart.checksum[0]));
            console->print(F("Sent baudrate message as:[ "));
            dumpBufferHex((uint8_t *)&cfgUart, sizeof(cfgUart));
            console->println(F("]"));
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


/**
 * read messages and call back with any message detected from the UbxMessageTypes
 * Configured.
 */
eUbloxMessageStatus UBXReader::read() {
    while (ubloxDevice.available() > 0) {
        uint8_t c = ubloxDevice.read();
        if ((c > 31 && c < 127) || c == '\r' || c == '\n') {
            console->write(c);
        } else {
            console->print("[0x");
            console->print(c,HEX);
            console->print("]");
        }
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
            if ( pos == (sizeof(UbloxHeader) + ubloxHeader->payloadLength + 2) ) {
                pos = 0;
                metrics.messageRecieved++;
                if ( pos < maxBufferSize ) {
                    if ( isChecksumCorrect() ) {
                        return msgStatusOk;
                    } else {
                        metrics.messageError++;
                        return msgStatusBadCheckSum;
                    }
                } else {
                    metrics.messageOverflow++;
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
    }
}

UbloxHeader *  UBXReader::getMessage() {
    return ubloxHeader;
}

bool UBXReader::isChecksumCorrect() {
  uint8_t b[] = {0,0};
  calculateChecksum(ubloxHeader, &b[0]);
  uint16_t messageLength =  ubloxHeader->payloadLength + sizeof(UbloxHeader);
  return (b[0] == packetBuffer[messageLength] && b[1] == packetBuffer[messageLength+1]);
}

void UBXReader::calculateChecksum(UbloxHeader *message, uint8_t *op) {
  uint8_t a = 0, b = 0;
  uint8_t *messageBuffer = (uint8_t *)message;
  uint16_t messageLength =  message->payloadLength + sizeof(UbloxHeader);
  // dont checksum the sync bytes or the checksum byte itself.
  for (uint16_t i = 2; i < messageLength; i++) {
    a =  a + messageBuffer[i];
    b =  b + a;
    console->print("idx:");
    console->print(i);
    console->print(" in:0x");
    console->print(messageBuffer[i], HEX);
    console->print(" a:");
    console->print(a);
    console->print("b:");
    console->println(b);
  }
  op[0] = a;
  op[1] = b;    
}

void UBXReader::sendMessage(UbloxHeader *message) {
  uint16_t messageLength =  message->payloadLength + sizeof(UbloxHeader);
  uint8_t *messageBuffer = (uint8_t *)message;
  calculateChecksum(message, &messageBuffer[messageLength]);
  ubloxDevice.write(messageBuffer,messageLength+2);
}
