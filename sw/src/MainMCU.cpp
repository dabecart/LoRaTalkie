#include "MainMCU.h"
#include "main.h"

MainMCU* mcu;

MainMCU::MainMCU(UART_HandleTypeDef* huart1,
                 DMA_HandleTypeDef* hdma_usart1_rx,
                 DMA_HandleTypeDef* hdma_usart1_tx,
                 ADC_HandleTypeDef* hadc1, DMA_HandleTypeDef* hdma_adc1,
                 DAC_HandleTypeDef* hdac1, DMA_HandleTypeDef* hdma_dac1_ch1,
                 TIM_HandleTypeDef* htim2) 
:
    uartLoRa(huart1, hdma_usart1_rx, hdma_usart1_tx),
    comms(&uartLoRa)
{
    mcu = this;

    this->hadc1 = hadc1;
    this->hdma_adc1 = hdma_adc1;
    this->hdac1 = hdac1;
    this->hdma_dac1_ch1 = hdma_dac1_ch1;
    this->htim2 = htim2;
}

void MainMCU::init() {
    uartLoRa.init();
    comms.init();

    // LoRa starts at 9600 (UART). Set to 115200 (know that the baudrate on "configuration" mode, 
    // that is, when reading registers, is always 9600 8N1). Also set the air data rate. 
    LoRaConfiguration config = comms.lora.currentConfig;
    config.speed.uartBaudRate = UART_BPS_115200;
    config.speed.airDataRate = AIR_DATA_RATE_111_62k5;
    config.channel = 18;
    config.transmissionMode.enableRSSI = RSSI_DISABLED;
    // Apply the configuration.
    if(comms.lora.writeConfigurationRegisters(config, 1) != LORA_SUCCESS) {
        Error_Handler();
    }

    // Now set the MCU's UART baudrate.
    if(!uartLoRa.setBaudrate(115200)) {
        Error_Handler();
    }

    changeWalkieState(currentlyTX);
    
    HAL_TIM_Base_Start(htim2);
}

MainMCU::~MainMCU() {}

void MainMCU::mainLoop() {
    debouncedStartTransmissionButton();

    if(currentlyTX) {
        // Check if there's enough data from the ADC ready to be sent to the LoRa.
        if(samplesBuf.len >= MESSAGE_SIZE) {
            HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_SET);
            uint8_t msgOut[MESSAGE_SIZE];
            samplesBuf.popN(MESSAGE_SIZE, msgOut);
            comms.sendMessage(COMMS_MULTICAST_ID, msgOut, MESSAGE_SIZE);
            HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
        }
    }else {
        // Check if there's data received from the LoRa.
        CommsMsg msg;
        if(comms.getNextMessage(&msg)) {
            HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_SET);
            samplesBuf.pushN(msg.payload, msg.header.getPayloadLength());
            
            // Start sending to the speaker when more than one message has been received.
            if(!startSendingToSpeaker) {
            	startSendingToSpeaker = samplesBuf.len > MESSAGE_SIZE;
            }
            
            HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
        }
    }

    uartLoRa.update();
}

void MainMCU::changeWalkieState(uint8_t startDoingTX) {
    // Disable all DMA transfers.
    HAL_ADC_Stop_DMA(hadc1);
    HAL_DAC_Stop_DMA(hdac1, DAC_CHANNEL_1);

    currentlyTX = startDoingTX;
    startSendingToSpeaker = 0;

    // Reset the buffers.
    memset(dmaBuf, 0, sizeof(dmaBuf));
    samplesBuf.empty();

    // Enable the peripherals depending on the current state of the device. 
    if(currentlyTX) {
        // Until the DMA fills either half of the buf, this number shall remain negative.
        dmaIndex = -1;
        // Mute the speaker.
        HAL_DAC_SetValue(hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0);
        // Start ADC to memory.
        HAL_ADC_Start_DMA(hadc1, (uint32_t*) dmaBuf, DMA_BUF_SIZE);
    }else {
        // DMA will start at index 0.
        dmaIndex = 0;
        // Start memory to DAC.
        HAL_DAC_Start_DMA(hdac1, DAC_CHANNEL_1, (uint32_t*) dmaBuf, DMA_BUF_SIZE, DAC_ALIGN_8B_R);
    }
}

void MainMCU::debouncedStartTransmissionButton() {
    static uint32_t lastStartTxLevelSwitchTime = 0;
    static GPIO_PinState lastStartTxLevel = GPIO_PIN_RESET;

    GPIO_PinState startTxPin = HAL_GPIO_ReadPin(START_TX_GPIO_Port, START_TX_Pin);
    if(startTxPin != lastStartTxLevel) {
        lastStartTxLevelSwitchTime = HAL_GetTick();
    }
    lastStartTxLevel = startTxPin;

    uint8_t newTXState = startTxPin == GPIO_PIN_SET;
    if((newTXState != currentlyTX) && ((HAL_GetTick() - lastStartTxLevelSwitchTime) >= 200)) {
        changeWalkieState(newTXState);
    }
}


/***************************************************************************************************
* ISR Callbacks
***************************************************************************************************/

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    uint8_t outBuf[SAMPLES_PER_BLOCK];
    uint16_t* dmaPointer = mcu->dmaBuf;
    for(uint32_t i = 0; i < SAMPLES_PER_BLOCK; i++) {
        outBuf[i] = (uint8_t) (*dmaPointer & 0x00FF);
        dmaPointer++;
    }
    mcu->samplesBuf.pushN(outBuf, SAMPLES_PER_BLOCK);

    mcu->dmaIndex = 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    uint8_t outBuf[SAMPLES_PER_BLOCK];
    uint16_t* dmaPointer = mcu->dmaBuf + SAMPLES_PER_BLOCK;
    for(uint32_t i = 0; i < SAMPLES_PER_BLOCK; i++) {
        outBuf[i] = (uint8_t) (*dmaPointer & 0x00FF);
        dmaPointer++;
    }
    mcu->samplesBuf.pushN(outBuf, SAMPLES_PER_BLOCK);

    mcu->dmaIndex = 1;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    // Wait until there are enough bytes for a block of data.
    if(!mcu->startSendingToSpeaker || mcu->samplesBuf.len < SAMPLES_PER_BLOCK) {
        // If not, clear the block.
        // This gives a warning, but it's ok.
        memset(mcu->dmaBuf, 0, SAMPLES_PER_BLOCK*sizeof(uint16_t));
        // There's no more data to send to the speaker.
        mcu->startSendingToSpeaker = 0;
    }

    if(!mcu->startSendingToSpeaker) return;

    // DMA has ended the first half, and has passed to the second half. Put data on the first half.
    uint8_t poppedBuf[SAMPLES_PER_BLOCK];
    mcu->samplesBuf.popN(SAMPLES_PER_BLOCK, poppedBuf);
    
    uint16_t* dmaPointer = mcu->dmaBuf;
    for(uint32_t i = 0; i < SAMPLES_PER_BLOCK; i++) {
        *dmaPointer = (uint16_t) poppedBuf[i];
        dmaPointer++;
    }

    mcu->dmaIndex = 1;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    // Wait until there are enough bytes for a block of data.
    if(!mcu->startSendingToSpeaker || mcu->samplesBuf.len < SAMPLES_PER_BLOCK) {
        // If not, clear the block.
        memset(mcu->dmaBuf + SAMPLES_PER_BLOCK, 0, SAMPLES_PER_BLOCK*sizeof(uint16_t));
        // There's no more data to send to the speaker.
        mcu->startSendingToSpeaker = 0;
        return;
    }

    // DMA has ended the second half, and has returned to the first half. Put data on the second 
    // half.
    uint8_t poppedBuf[SAMPLES_PER_BLOCK];
    mcu->samplesBuf.popN(SAMPLES_PER_BLOCK, poppedBuf);
    
    uint16_t* dmaPointer = mcu->dmaBuf + SAMPLES_PER_BLOCK;
    for(uint32_t i = 0; i < SAMPLES_PER_BLOCK; i++) {
        *dmaPointer = (uint16_t) poppedBuf[i];
        dmaPointer++;
    }
    
    mcu->dmaIndex = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == LORA_AUX_Pin) mcu->comms.lora.auxPinISR();
}
