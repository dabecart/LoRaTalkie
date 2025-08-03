#include "CCalls.h"
#include "MainMCU.h"

MainMCU* mainMCU;

void createMainMCU(UART_HandleTypeDef* huart1,
                   DMA_HandleTypeDef*  hdma_usart1_rx,
                   DMA_HandleTypeDef*  hdma_usart1_tx,
                   ADC_HandleTypeDef* hadc1, DMA_HandleTypeDef* hdma_adc1,
                   DAC_HandleTypeDef* hdac1, DMA_HandleTypeDef* hdma_dac1_ch1,
                   TIM_HandleTypeDef* htim2) 
{
    mainMCU = new MainMCU(huart1, hdma_usart1_rx, hdma_usart1_tx, 
                          hadc1, hdma_adc1, hdac1, 
                          hdma_dac1_ch1, htim2);
}

void initMainMCU() {
    mainMCU->init();
}

void loopMainMCU() {
    mainMCU->mainLoop();
}
