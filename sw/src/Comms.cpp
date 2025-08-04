#include "Comms.h"

Comms::Comms(UART* uart) : lora(uart) {}

Comms::~Comms() {}

void Comms::init() {
    lora.init();
}

uint8_t Comms::getNextMessage(CommsMsg* msg) {
    // TODO: Implement the package split done automatically by the LoRa module.

    if(msg == NULL) return 0;

    if(lora.uart->RXBuffer.len < COMMS_HEAD_LEN) return 0;

    while(lora.uart->RXBuffer.peekN(COMMS_HEAD_LEN, msg->header.head) && 
          !msg->header.isHeadValid()) {
        // No header found. Pop the first byte.
        lora.uart->RXBuffer.pop(NULL);
    }

    if(!msg->header.isHeadValid()) return 0;

    // Head of message found!
    if(!lora.uart->RXBuffer.peekN(COMMS_HEADER_LEN, (uint8_t*) &msg->header)) {
        // Not enough bytes to peek.
        return 0;
    }

    // Header found!
    if((msg->header.receiverID != COMMS_MULTICAST_ID) && (msg->header.receiverID != id)) {
        // This message is not for this device.
        lora.uart->RXBuffer.pop(NULL);
        return 0;
    }

    if(lora.uart->RXBuffer.len < msg->header.length) {
        // Not enough bytes to get the payload.
        return 0;
    }

    // Full message in memory!
    uint16_t payloadLength = msg->header.getPayloadLength();
    msg->payload = (uint8_t*) malloc(payloadLength);
    if(msg->payload == NULL) {
        // Could not allocate the payload in memory.
        lora.uart->RXBuffer.pop(NULL);
        return 0;
    }

    lora.uart->RXBuffer.popN(COMMS_HEADER_LEN, (uint8_t*) &msg->header);
    lora.uart->RXBuffer.popN(payloadLength, msg->payload);
    lora.uart->RXBuffer.popN(COMMS_CRC_LEN, (uint8_t*) &msg->crc);

    // Full message stored in msg.
    uint16_t crc = calculateCRC((uint8_t*) &msg->header, COMMS_HEADER_LEN);
    crc = calculateCRCFromStartingValue(
        msg->payload, payloadLength, crc);
    if(crc != msg->crc) {
        // CRC is not valid.
        crcErrorCount++;
        lora.uart->RXBuffer.pop(NULL);
        return 0;
    }

    // Found a valid message!
    if(lora.currentConfig.transmissionMode.enableRSSI) {
        // The RSSI is located following the end of the message.
        // RSSI is calculated as -(256 - field)
        uint8_t rssiField = 0;
        // TODO: If there aren't enough bytes, this line will fail.
        lora.uart->RXBuffer.pop(&rssiField);
        if(rssiField == 0) {
            // I don't know why but field = 0 -> RSSI = 0.
            msg->rssi = 0;
        }else {
            msg->rssi = ((int16_t) rssiField) - 256;
        }
    }else {
        msg->rssi = 0;
    }

    return 1;
}

uint8_t Comms::sendMessage(uint16_t addrs, uint8_t* data, uint16_t dataLen) {
    if((data == NULL) && (dataLen > 0)) return 0;

    CommsMsgHeader header = {
        .head = COMMS_HEADER,
        .length = (uint16_t) (COMMS_HEADER_LEN + dataLen  + COMMS_CRC_LEN),
        .sourceID = id,
        .receiverID = addrs
    };

    uint16_t crc = calculateCRC((uint8_t*) &header, COMMS_HEADER_LEN);
    crc = calculateCRCFromStartingValue(data, dataLen, crc);

    uint8_t* outBuffer = (uint8_t*) malloc(header.length);
    if(outBuffer == NULL) {
        // Not enough memory.
        return 0;
    }

    memcpy(outBuffer, (uint8_t*) &header, COMMS_HEADER_LEN);
    memcpy(outBuffer+COMMS_HEADER_LEN, data, dataLen);
    memcpy(outBuffer+COMMS_HEADER_LEN+dataLen, &crc, COMMS_CRC_LEN);

    lora.setMode(LoRaMode::MODE_0_NORMAL);
    lora.writeDataDMA(outBuffer, header.length);

    free(outBuffer);

    return 1;
}

uint16_t Comms::calculateCRC(uint8_t *bufferInput, uint16_t bufferSize) 
{
    if((bufferInput == NULL) || (bufferSize <= 0)) {
        return -1;
    }
    
    // The initial word, if no starting word is given, is 0xFFFF.
    return calculateCRCFromStartingValue(bufferInput, bufferSize, 0xFFFF);
}

uint16_t Comms::calculateCRCFromStartingValue(
       uint8_t *bufferInput, uint16_t bufferLen, uint16_t startCRC) {
    uint16_t usCRC = startCRC;

    // No inversion is applied to the input.

    for (uint16_t ulByteIndex = 0; ulByteIndex < bufferLen; ++ulByteIndex) {
        usCRC ^= (uint16_t)(bufferInput[ulByteIndex] << 8);
        for (int ulBitIndex = 0; ulBitIndex < 8; ++ulBitIndex) {
            if (usCRC & 0x8000) {
                usCRC = (usCRC << 1) ^ 0x1021;  // 0x1021: CRC polynomium.
            }else {
                usCRC <<= 1;
            }
        }
    }

    // No inversion nor final XOR is applied to the output.
    return usCRC;
}
