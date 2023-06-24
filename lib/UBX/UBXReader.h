#pragma once


#include <Arduino.h>
#include "UBXMessages.h"

#define ubloxDevice Serial

enum eUbloxMessageStatus {
    msgStatusOk = 0,
    msgStatusBadCheckSum = 1,
    msgStatusOverflow = 2,
    msgIncomplete = 3
};

typedef struct _UBXReaderMetrics {
    uint16_t bytesRead = 0;
    uint16_t bufferRestarts = 0;
    uint16_t messageRecieved = 0;
    uint16_t messageError = 0;
    uint16_t messageOverflow = 0;
} UBXReaderMetrics;
/**
 * A reader for UBX messages
 * It will read messages and call back when a recognised message is detected.
 * The callback contains the message type, a pointer to the packetBuffer and the 
 * packetBuffer length.
 */
class UBXReader {
public:
    /**
     * Create a reader with a buffer for holding messages.
     * Messages that are longer than the buffer will be truncated when completely recieved
     * indicated by a msgStatusOverflow return from read.
     */
    UBXReader(Stream *console, uint8_t * buffer, uint16_t len); 
    /**
     * read bytes from the stream and return the a status flag indicating the state of the message.
     * 
     */
    eUbloxMessageStatus read();
    /**
     * Get the current message in the buffer after msyStatusOk is reported.
     */
    UbloxHeader * getMessage();
    /**
     * Get metrics
     */
    UBXReaderMetrics * getMetrics() {
        return &metrics;
    }
    bool begin();
private:
    Stream * console;
    uint8_t * packetBuffer; 
    UbloxHeader * ubloxHeader;
    uint16_t maxBufferSize;

    uint16_t pos = 0;
    UBXReaderMetrics metrics;
    bool isChecksumCorrect();
    void calculateChecksum(UbloxHeader *message, uint8_t *op);
    void sendMessage(UbloxHeader *message);
    void dumpBufferHex(uint8_t * buffer, uint16_t len);
    void setMessageRate(uint8_t cls, uint8_t id, uint8_t rate);
    void setNavRate(uint16_t measMs, uint16_t measCycles, uint16_t ref);

};