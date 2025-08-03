/***************************************************************************************************
 * @file UART.h
 * @brief Handler for serial communications over UART.
 * 
 * @version 1.0
 * @date    2024-12-07
 * @author  @dabecart
 * 
 * @license This project is licensed under the MIT License - see the LICENSE file for details.
***********************************************************************************************/
#ifndef UART_h
#define UART_h

#include "CircularBuffer.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dma.h"
#include "Defines.h"

class UART {
    public:
    /**
     * @brief Construct a new UART object.
     * 
     * @param hUART. The UART handler.
     * @param rxDMA. The DMA handler of the RX.
     * @param txDMA. The DMA hablder of the TX. 
     */
    UART(
        UART_HandleTypeDef*     hUART,
        DMA_HandleTypeDef*      rxDMA,
        DMA_HandleTypeDef*      txDMA
    );

    /**
     * @brief Destroy the UART object.
     */
    ~UART();

    /**
     * @brief Attaches the DMA ports.
     */
    void init();

    /**
     * @brief Update the UART RX buffers.
     */
    void update();

    /**
     * @brief Send a buffer to an UART.
     * 
     * @param pucMessage. Message to send.
     * @param usMessageLength. The message's number of bytes.
     * @return uint8_t 1 if sent correctly.
     */
    uint8_t sendToUART(uint8_t* pucMessage, uint16_t usMessageLength);

    /**
     * @brief Links a DMA to the circular buffers of the serial port.
     */
    void attachDMAToSerialPort();

    public:
    CircularBuffer<uint8_t, UART_BUF_SIZE>   RXBuffer;
    CircularBuffer<uint8_t, UART_BUF_SIZE>   TXBuffer;
    UART_HandleTypeDef*                      hUART;
    DMA_HandleTypeDef*                       rxDMA;
    DMA_HandleTypeDef*                       txDMA;
    uint8_t                                  txSend = 0;    // 1 when a message is ready to be sent
    uint8_t                                  rxSend = 0;    // 1 when a message has been received
};

#endif // UART_h