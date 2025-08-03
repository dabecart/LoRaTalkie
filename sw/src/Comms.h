#ifndef COMMS_h
#define COMMS_h

#include <stdlib.h>
#include "UART.h"
#include "LoRa.h"

#define COMMS_HEADER (uint8_t[]){'$', '#'}
#define COMMS_HEAD_LEN sizeof(COMMS_HEADER)
#define COMMS_HEADER_LEN sizeof(CommsMsgHeader)
#define COMMS_CRC_LEN 2

#define COMMS_MULTICAST_ID (uint32_t) 0xFFFF

typedef struct CommsMsgHeader { 
    uint8_t head[COMMS_HEAD_LEN];
    uint16_t length;
    uint32_t sourceID;
    uint32_t receiverID;

    uint8_t isHeadValid() {
        return (head[0] == COMMS_HEADER[0]) && (head[1] == COMMS_HEADER[1]);
    }

    uint16_t getPayloadLength() {
        return length - COMMS_HEADER_LEN - COMMS_CRC_LEN;
    }
} __attribute__((__packed__)) CommsMsgHeader;

typedef struct CommsMsg {
    CommsMsgHeader header;
    uint8_t* payload = NULL;
    uint16_t crc;
    int16_t rssi;

    ~CommsMsg(){
        if(payload != NULL) free(payload);
    }
} CommsMsg;

class Comms {
    public:
    Comms(UART* uart);
    ~Comms();

    void init();

    uint8_t getNextMessage(CommsMsg* msg);

    uint8_t sendMessage(uint16_t addrs, uint8_t* data, uint16_t dataLen);

    private:
    uint16_t calculateCRC(uint8_t *bufferInput, uint16_t bufferLen);

    uint16_t calculateCRCFromStartingValue(
        uint8_t *bufferInput, uint16_t bufferLen, uint16_t startCRC);

    public:
    LoRa lora;
    uint32_t id = COMMS_MULTICAST_ID;
    uint16_t crcErrorCount = 0;
};

#endif // COMMS_h