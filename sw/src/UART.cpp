/***************************************************************************************************
 * @file UART.h
 * @brief Handler for serial communications over UART.
 * 
 * @version 1.0
 * @date    2024-12-07
 * @author  @dabecart
 * 
 * @license This project is licensed under the MIT License - see the LICENSE file for details.
***************************************************************************************************/
#include "UART.h"
#include "MainMCU.h"

UART::UART(UART_HandleTypeDef*     hUART,
           DMA_HandleTypeDef*      rxDMA,
           DMA_HandleTypeDef*      txDMA) {

    this->hUART        = hUART;
    this->rxDMA        = rxDMA; 
    this->txDMA        = txDMA;
}

UART::~UART() {}

void UART::init() {
    attachDMAToSerialPort();
}

void UART::update() {
    if(txSend) {
        // Lock the buffer until the DMA has sent the message. The callback will unlock it.
        TXBuffer.locked = 1;

        HAL_UART_Transmit_DMA(hUART, TXBuffer.data, TXBuffer.len);

        // Disables an interruption that gets called half in transmission.
        __HAL_DMA_DISABLE_IT(txDMA, DMA_IT_HT);
        
        // The buffer has been sent!
        txSend = 0;
    }
}

uint8_t UART::sendToUART(uint8_t* pucMessage, uint16_t usMessageLength) {
    // Push the data to the array.
    uint8_t readyToSend = TXBuffer.pushN(pucMessage, usMessageLength);
    if(readyToSend) txSend = 1;
    return readyToSend;
}

uint8_t UART::setBaudrate(uint32_t baudrate) {
    uint8_t ret = 1;
    ret &= HAL_UART_DeInit(hUART) == HAL_OK;
    hUART->Init.BaudRate = baudrate;
    ret &= HAL_UART_Init(hUART) == HAL_OK;
    attachDMAToSerialPort();
    return ret;
}

void UART::attachDMAToSerialPort() {
    // This function takes the buffer directly as a circular one. It won't update the indices of the 
    // struct, so that will be left upon us. 
    HAL_UARTEx_ReceiveToIdle_DMA(hUART, RXBuffer.data, RXBuffer.size);

    // // This disables an interruption that triggers when the buffer gets filled to its full size.
    // __HAL_DMA_DISABLE_IT(pstUART->rxDMA, DMA_IT_TC);
    // // This disables an interruption that triggers when the buffer gets filled to half its size.
    // __HAL_DMA_DISABLE_IT(pstUART->rxDMA, DMA_IT_HT);
}

/***************************************************************************************************
* ISR Callbacks
***************************************************************************************************/

/***************************************************************************************************
* This function is declared inside stm32l4xx_hal_uart.h for the user to define it. 
* This function gets called when a reception over a serial port has ended.
***************************************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *hUART, uint16_t newHeadIndex) {   
    UART* referencedUART = NULL;
    if(hUART == mcu->uartLoRa.hUART) {
        referencedUART = &mcu->uartLoRa;
    } // Add more UART handlers if used.

    if(referencedUART != NULL) {
        referencedUART->RXBuffer.updateIndices(newHeadIndex);
        referencedUART->rxSend = 1;
    }
}

/***************************************************************************************************
* This function is declared inside stm32l4xx_hal_uart.h for the user to define it. 
* This function gets called when a transmission over a serial port has ended.
***************************************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hUART) {
    UART* referencedUART = NULL;
    if(hUART == mcu->uartLoRa.hUART) {
        referencedUART = &mcu->uartLoRa;
    } // Add more UART handlers if used.

    if(referencedUART != NULL) {
        // Unlock the TX buffer.
        referencedUART->TXBuffer.locked = 0;
        // The transmission doesn't use circular buffers, but we're using them so that we don't have
        // to implement a "simple buffer". To convert a circular buffer to simple buffer, just 
        // delete its content and restart the indices.
        referencedUART->TXBuffer.empty();
    }
}

/***************************************************************************************************
* This function is declared inside stm32l4xx_hal_uart.h for the user to define it. 
* This function gets called when an UART fails.
***************************************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef* hUART) {
    UART* referencedUART = NULL;
    if(hUART == mcu->uartLoRa.hUART) {
        referencedUART = &mcu->uartLoRa;
    } // Add more UART handlers if used.

    if(referencedUART != NULL) {
        referencedUART->TXBuffer.empty();
        referencedUART->TXBuffer.locked = 0;
        referencedUART->txSend = 0;

        referencedUART->RXBuffer.empty();
        referencedUART->RXBuffer.locked = 0;
        referencedUART->rxSend = 0;

        referencedUART->attachDMAToSerialPort();
    }
}

