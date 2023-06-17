
#include "UBXReader.h"


/**
 * Call the UBX reader with a list of UbxMessageTypes
 */
UBXReader::UBXReader(UartClass * serialIo, const uint8_t * buffer, uint16_t len) {
    io = serialIo;
    packetBuffer = buffer;
    ubloxHeader = (UbloxHeader *) buffer;
    maxBufferSize = len;
};
/**
 * read messages and call back with any message detected from the UbxMessageTypes
 * Configured.
 */
eUbloxMessageStatus UBXReader::read() {
    if (io->available()) {
        uint8_t c = io->read();
        if ( pos < maxBufferSize ) {
            packetBuffer[pos] = c;
        }
        pos++;
        if ( pos == 1 ) {
            if ( c != ULBOX_SYNC1 ){
                bufferRestarts++;
                pos = 0;                
            } 
        } else if (pos == 2) {
            if ( c != ULBOX_SYNC2 ){
                bufferRestarts++;
                pos = 0;                
            } 
        } else if ( pos >= sizeof(UbloxHeader) ) {
            if ( pos == (sizeof(UbloxHeader) + ubloxHeader->payloadLength + 1) ) {
                pos = 0;
                messageRecieved++;
                if ( pos < maxBufferSize ) {
                    if ( isChecksumCorrect() ) {
                        return msgStatusOk;
                    } else {
                        messageError++;
                        return msgStatusBadCheckSum;
                    }
                } else {
                    messageOverflow++;
                    return msgStatusOverflow;
                }
            }
        }
    }
    return msgIncomplete;
}

UbloxHeader *  UBXReader::getMessage() {
    return ubloxHeader;
}

bool UBXReader::isChecksumCorrect() {
  uint8_t b[] = {0,0};
  uint16_t messageLength =  ubloxHeader->payloadLength + sizeof(UbloxHeader);
  // dont checksum the sync bytes or the checksum byte itself.
  for (uint16_t i = 2; i < messageLength; i++) {
    b[0] += packetBuffer[i];
    b[1] += b[0];
  }
  return (b[1] == packetBuffer[messageLength]);
}
