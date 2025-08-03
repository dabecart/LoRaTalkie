#ifndef C_CALLS_h
#define C_CALLS_h

// C Calls is a header file used to connect the C code with the C++ code.

// This header is used as an static instance of an SD card. This way all funcions related to FATFS
// (which are written in C) do not have to be converted to C++. This is so that the autogeneration
// of code does not mess with the files.

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void createMainMCU(UART_HandleTypeDef* huart1,
                   DMA_HandleTypeDef*  hdma_usart1_rx,
                   DMA_HandleTypeDef*  hdma_usart1_tx,
                   ADC_HandleTypeDef* hadc1, DMA_HandleTypeDef* hdma_adc1,
                   DAC_HandleTypeDef* hdac1, DMA_HandleTypeDef* hdma_dac1_ch1,
                   TIM_HandleTypeDef* htim2);
void initMainMCU();
void loopMainMCU();

#ifdef __cplusplus
}
#endif

#endif // C_CALLS_h