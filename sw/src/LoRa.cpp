#include "LoRa.h"
#include "MainMCU.h"

LoRa::LoRa(UART* uart) : uart(uart) {}

LoRa::~LoRa() {}

LoRaStatus LoRa::init() {
    LoRaStatus configSt = readConfigurationRegisters(&currentConfig);
    LoRaStatus modeSt = setMode(LoRaMode::MODE_0_NORMAL);
    if(configSt != LoRaStatus::LORA_SUCCESS) {
        return configSt;
    }else if(modeSt != LoRaStatus::LORA_SUCCESS) {
        return modeSt;
    }else {
        return LoRaStatus::LORA_SUCCESS;
    }
}

LoRaStatus LoRa::readConfigurationRegisters(LoRaConfiguration* config) {
    if(config == NULL) return LoRaStatus::LORA_ERR_INVALID_PARAM;
    // Mode 3 only uses 9600 8N1.
    if(uart->hUART->Init.BaudRate != 9600) return LoRaStatus::LORA_ERR_HARDWARE;

    LoRaMode previousMode = currentMode;

    // Switch to Program mode.
    LoRaStatus st = setMode(LoRaMode::MODE_3_CONFIGURATION);
    if(st != LoRaStatus::LORA_SUCCESS) return st;

    // Write the command to read the configuration register (CFG).
    uint8_t writeStatus = writeProgramCommand(LoRaCommand::READ_CONFIGURATION, 
                                              LoRaRegAdds::REG_ADDS_ADDH, 
                                              LoRaPacketLength::PL_CONFIGURATION);
    if(!writeStatus) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_HARDWARE;        
    }

    // Receive response.
    LoRaStatus receiveStatus = receiveData(sizeof(LoRaConfiguration), (uint8_t*) config);
    if(receiveStatus != LoRaStatus::LORA_SUCCESS) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_HARDWARE;
    }

    // Check the command field of the response.
    if(config->command == LoRaCommand::WRONG_FORMAT) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_WRONG_FORMAT;
    }else if(config->command != LoRaCommand::READ_CONFIGURATION ||
             config->startAddress != LoRaRegAdds::REG_ADDS_ADDH ||
             config->length != LoRaPacketLength::PL_CONFIGURATION) {
        return LoRaStatus::LORA_ERR_HEAD_NOT_RECOGNIZED;
    }

    // Switch to the previous mode.
    setMode(previousMode);

    return LoRaStatus::LORA_SUCCESS;
}

LoRaStatus LoRa::writeConfigurationRegisters(LoRaConfiguration config, uint8_t temporarySave) {
    // Mode 3 only uses 9600 8N1.
    if(uart->hUART->Init.BaudRate != 9600) return LoRaStatus::LORA_ERR_HARDWARE;

    LoRaMode previousMode = currentMode;

    // Switch to Program mode.
    LoRaStatus st = setMode(LoRaMode::MODE_3_CONFIGURATION);
    if(st != LoRaStatus::LORA_SUCCESS) return st;

    // Write the configuration registers (CFG).
    config.command = temporarySave ? LoRaCommand::WRITE_CFG_PWR_DWN_LOSE : 
                                     LoRaCommand::WRITE_CFG_PWR_DWN_SAVE;
    config.startAddress = LoRaRegAdds::REG_ADDS_ADDH;
    config.length = LoRaPacketLength::PL_CONFIGURATION;

    LoRaStatus writeStatus = writeData((uint8_t*) &config, sizeof(LoRaConfiguration));
    if(!writeStatus) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_HARDWARE;        
    }

    // The device should return header + payload (11 bytes).
    // The module should respond with the same payload as the configuration register.
    LoRaConfiguration readConfig;
    LoRaStatus receiveStatus = receiveData(sizeof(readConfig), (uint8_t*) &readConfig);
    if(receiveStatus != LoRaStatus::LORA_SUCCESS) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_HARDWARE;        
    }

    // Check the command field of the response.
    if(readConfig.command == LoRaCommand::WRONG_FORMAT) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_WRONG_FORMAT;
    }

    // Check that received payload is the same as the sent payload. 
    // Only check from addressH to transmission mode.
    uint8_t* wConf = ((uint8_t*) &config) + LORA_HEADER_LEN;
    uint8_t* rConf = ((uint8_t*) &readConfig) + LORA_HEADER_LEN;
    LoRaStatus checkStatus = LoRaStatus::LORA_SUCCESS;
    if(
        (wConf[LoRaRegAdds::REG_ADDS_ADDH]       != rConf[LoRaRegAdds::REG_ADDS_ADDH])       ||
        (wConf[LoRaRegAdds::REG_ADDS_ADDL]       != rConf[LoRaRegAdds::REG_ADDS_ADDL])       ||
        (wConf[LoRaRegAdds::REG_ADDS_SPED]       != rConf[LoRaRegAdds::REG_ADDS_SPED])       ||
        (wConf[LoRaRegAdds::REG_ADDS_TRANS_MODE] != rConf[LoRaRegAdds::REG_ADDS_TRANS_MODE]) ||            
        (wConf[LoRaRegAdds::REG_ADDS_CHANNEL]    != rConf[LoRaRegAdds::REG_ADDS_CHANNEL])    ||    
        (wConf[LoRaRegAdds::REG_ADDS_OPTION]     != rConf[LoRaRegAdds::REG_ADDS_OPTION])    
    ){
        checkStatus = LoRaStatus::LORA_ERR_INVALID_RESPONSE;
    }

    // Switch to the previous mode.
    setMode(previousMode);

    return checkStatus;
}

LoRaStatus LoRa::getModuleInformation(LoRaPID* pid) {
    if(pid == NULL) return LoRaStatus::LORA_ERR_INVALID_PARAM;
    // Mode 3 only uses 9600 8N1.
    if(uart->hUART->Init.BaudRate != 9600) return LoRaStatus::LORA_ERR_HARDWARE;

    LoRaMode previousMode = currentMode;

    // Switch to Program mode.
    LoRaStatus st = setMode(LoRaMode::MODE_3_CONFIGURATION);
    if(st != LoRaStatus::LORA_SUCCESS) return st;

    // Write the command to read the configuration register (CFG).
    uint8_t writeStatus = writeProgramCommand(LoRaCommand::READ_CONFIGURATION, 
                                              LoRaRegAdds::REG_ADDS_PID, 
                                              LoRaPacketLength::PL_PID);
    if(!writeStatus) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_HARDWARE;        
    }

    // Receive response.
    LoRaStatus receiveStatus = receiveData(sizeof(LoRaPID), (uint8_t*) &pid);
    if(receiveStatus != LoRaStatus::LORA_SUCCESS) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_HARDWARE;
    }

    // Check the command field of the response.
    if(pid->command == LoRaCommand::WRONG_FORMAT) {
        setMode(previousMode);
        return LoRaStatus::LORA_ERR_WRONG_FORMAT;
    }

    // Switch to the previous mode.
    setMode(previousMode);

    return LoRaStatus::LORA_SUCCESS;
}

LoRaStatus LoRa::receiveData(uint16_t dataLen, uint8_t* dataBuffer) {
    // While the LoRa module is transmitting the AUX pin remains LOW.
    LoRaStatus auxStatus = waitAUXPin(1000);
    HAL_Delay(1);
    // Read data from the UART circular buffer.
    uint8_t dataOk = uart->RXBuffer.popN(dataLen, dataBuffer);
    if(!dataOk) return LoRaStatus::LORA_ERR_INVALID_RESPONSE;
    return auxStatus; 
}

LoRaStatus LoRa::writeData(uint8_t* dataBuffer, uint16_t dataLen) {
    if(dataBuffer == NULL || dataLen == 0) return LORA_ERR_INVALID_PARAM;
    
    // Write data to the UART.
    HAL_StatusTypeDef status = HAL_UART_Transmit(uart->hUART, dataBuffer, dataLen, 1000);
    // While the LoRa module is transmitting the AUX pin remains LOW.
    LoRaStatus auxStatus = waitAUXPin(1000);
    if(status == HAL_TIMEOUT) return LoRaStatus::LORA_ERR_TIMEOUT;
    else if(status == HAL_ERROR) return LoRaStatus::LORA_ERR_UNKNOWN;
    return auxStatus; 
}

LoRaStatus LoRa::writeDataDMA(uint8_t* dataBuffer, uint16_t dataLen) {
    if(dataBuffer == NULL || dataLen == 0) return LORA_ERR_INVALID_PARAM;

    uint16_t initialPacketLen = dataLen;
    if(currentlySendingThroughDMA) {
        // If there is pending data to be sent, append to the buffer.
        dmaBuf.pushN(dataBuffer, dataLen);
    }else {
        if(dataLen > LORA_MAX_MSG_LEN) {
            // Data will have to be split and sent sequentially.
        	initialPacketLen = LORA_MAX_MSG_LEN;
    
            dmaBuf.pushN(dataBuffer + LORA_MAX_MSG_LEN, dataLen - LORA_MAX_MSG_LEN);
            currentlySendingThroughDMA = 1;
        }

        // Send the initial packet.
        if(!uart->sendToUART(dataBuffer, initialPacketLen)) {
            return LORA_ERR_UNKNOWN;
        }
    }

    return LORA_SUCCESS;
}

void LoRa::auxPinISR() {
    if(!currentlySendingThroughDMA) return;

    if(!uart->txSend && !uart->TXBuffer.locked) {
        // UART has finished sending the previous packet through DMA.
        uint8_t msg[LORA_MAX_MSG_LEN];
        uint16_t len;
        if(dmaBuf.len > LORA_MAX_MSG_LEN) {
            len = LORA_MAX_MSG_LEN;
        }else {
            len = dmaBuf.len;
            // Finished sending the last packet!
            currentlySendingThroughDMA = 0;
        }

        dmaBuf.popN(len, msg);
        uart->sendToUART(msg, len);
    }
    
    // If the UART is still sending data, then it is expected that the AUX pin will have to rise 
    // once the UART ends its transmission, therefore calling this interrupt again.
}

LoRaStatus LoRa::waitAUXPin(uint32_t timeout_ms) {
    // The AUX pin remains LOW while the module is busy.
    uint32_t t0 = HAL_GetTick();
    while(HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
        if(((uint32_t)(HAL_GetTick() - t0)) > timeout_ms) {
            return LoRaStatus::LORA_ERR_TIMEOUT;
        }
    }

    HAL_Delay(20);
    return LoRaStatus::LORA_SUCCESS;
}

uint8_t LoRa::writeProgramCommand(
    LoRaCommand cmd, LoRaRegAdds addrs, LoRaPacketLength packetLength) {
    uint8_t outMsg[3] = {cmd, addrs, packetLength};
    HAL_StatusTypeDef status = HAL_UART_Transmit(uart->hUART, outMsg, sizeof(outMsg), 1000);
    HAL_Delay(50);
    return status == HAL_OK;
}

LoRaStatus LoRa::setMode(LoRaMode mode) {
    if(currentMode == mode) return LoRaStatus::LORA_SUCCESS;
    
    HAL_Delay(40);

    switch (mode) {
        case MODE_0_NORMAL:
            HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
            break;
        case MODE_1_WOR_TRANSMITTER:
            HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
            break;
        case MODE_2_WOR_RECEIVER:
            HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);
            break;
        case MODE_3_CONFIGURATION:
            HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);
            break;

        default:
            return LORA_ERR_INVALID_PARAM;
    }

    HAL_Delay(40);

    LoRaStatus st = waitAUXPin(1000);
    if(st == LoRaStatus::LORA_SUCCESS) {
        currentMode = mode;
    } 
    
    // Empty the buffer upon changing mode (this gets rid of previous messages).
    uart->RXBuffer.popN(uart->RXBuffer.len, NULL);

    return st;
}
