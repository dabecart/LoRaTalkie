#ifndef MAIN_MCU_h
#define MAIN_MCU_h

#include "stm32g4xx_hal.h"
#include "UART.h"
#include "Comms.h"
#include "Defines.h"

#include "adpcm.h"

class MainMCU {
    public:
    /**
     * @brief Construct a new MainMCU object.
     * 
     * @param huart1. Handler to UART1.
     * @param hdma_usart1_rx. Handler to the DMA_UART1_RX.
     * @param hdma_usart1_tx. Handler to the DMA_UART1_TX.
     * @param hadc1. Hanlder to ADC1. 
     * @param hdma_adc1. Hanlder to DMA_ADC1.
     * @param hdac1. Hanlder to DAC1.
     * @param hdma_dac1_ch1. Hanlder to DMA_DAC1.
     * @param htim2. Hanlder to TIM2.
     */
    MainMCU(UART_HandleTypeDef* huart1,
            DMA_HandleTypeDef*  hdma_usart1_rx,
            DMA_HandleTypeDef*  hdma_usart1_tx,
            ADC_HandleTypeDef* hadc1, DMA_HandleTypeDef* hdma_adc1,
            DAC_HandleTypeDef* hdac1, DMA_HandleTypeDef* hdma_dac1_ch1,
            TIM_HandleTypeDef* htim2);

    /**
     * @brief Destroy the MainMCU object.
     */
    ~MainMCU();

    void init();

    /**
     * @brief Main routine of the MCU. 
     */
    void mainLoop();

    private:
    void changeWalkieState(uint8_t startDoingTX);
    void debouncedStartTransmissionButton();

    public:
    UART uartLoRa;
    Comms comms;

    ADC_HandleTypeDef* hadc1;
    DMA_HandleTypeDef* hdma_adc1;
    DAC_HandleTypeDef* hdac1;
    DMA_HandleTypeDef* hdma_dac1_ch1;
    TIM_HandleTypeDef* htim2;

    uint16_t dmaBuf[DMA_BUF_SIZE];
    CircularBuffer<uint8_t, UART_BUF_SIZE> samplesBuf;

    uint8_t currentlyTX = 0;
    // Points to where the DMA currently is. If <0, discard the value.
    int8_t dmaIndex = -1;
    // When currentlyTX = 0, when more than one message has been received, start sending to the 
    // speaker.
    uint8_t startSendingToSpeaker = 0;
};

extern MainMCU* mcu;

#endif // MAIN_MCU_h