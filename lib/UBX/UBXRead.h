#pragma once

#define ULBOX_SYNC1 0x5B
#define ULBOX_SYNC2 0x62

typedef struct _UbloxHeader {
    uint8_t sync1; // 0x5B
    uint8_t sync2; // 0x62
    uint8_t messageClass;
    uint8_t messageId;
    uint16_t payloadLength;
} UbloxHeader;



typedef struct _ubx_message_type {
    unsigned char cls;
    unsigned char id;
    uint16_t size;
} UbxMessageType;

enum eUbloxMessageStatus {
    msgStatusOk = 0;
    msgStatusBadCheckSum = 1;
    msgStatusOverflow = 2;
    msgIncomplete = 3;
}
/**
 * A reader for UBX messages
 * It will read messages and call back when a recognised message is detected.
 * The callback contains the message type, a pointer to the packetBuffer and the 
 * packetBuffer length.
 */
class UBXReader {
public:
    /**
     * Call the UBX reader with a list of UbxMessageTypes
     */
    UBXReader(UartClass * serialio, uint8_t * buffer, uint16_t len); 
    /**
     * read messages and call back with any message detected from the UbxMessageTypes
     * Configured.
     */
    eUbloxMessageStatus read();
    /**
     * 
     */
    UbloxHeader * getMessage();
private:
    unsigned char * packetBuffer; 
    UbloxHeader * ubloxHeader;
    int pos = 0;
    bool isChecksumCorrect();

}