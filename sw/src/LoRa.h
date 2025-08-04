#ifndef LORA_h
#define LORA_h

#include <stdint.h>
#include <string.h>
#include "stm32g4xx_hal.h"
#include "main.h"
#include "UART.h"

#define LORA_BROADCAST_ADDRESS 255
#define LORA_HEADER_LEN 3   // command, startAddress, length
// Cannot put the maximum (200) it hangs the device.
#define LORA_MAX_MSG_LEN 190

/**
 * @brief Return field for LoRa functions. 
 */
enum LoRaStatus {
  LORA_SUCCESS = 1,
  LORA_ERR_UNKNOWN,
  LORA_ERR_INVALID_PARAM,
  LORA_ERR_TIMEOUT,
  LORA_ERR_HARDWARE,
  LORA_ERR_HEAD_NOT_RECOGNIZED,
  LORA_ERR_WRONG_FORMAT,
  LORA_ERR_INVALID_RESPONSE,
  LORA_ERR_NO_NEW_MSG
};

enum LoRaUARTParity {
  MODE_00_8N1 = 0b00,
  MODE_01_8O1 = 0b01,
  MODE_10_8E1 = 0b10,
  MODE_11_8N1 = 0b11
};

enum LoRaUARTBaudrate {
  UART_BPS_1200 = 0b000,
  UART_BPS_2400 = 0b001,
  UART_BPS_4800 = 0b010,
  UART_BPS_9600 = 0b011,
  UART_BPS_19200 = 0b100,
  UART_BPS_38400 = 0b101,
  UART_BPS_57600 = 0b110,
  UART_BPS_115200 = 0b111
};

enum LoRaAirDataRate {
  AIR_DATA_RATE_000_2k4 = 0b000,
  AIR_DATA_RATE_001_2k4 = 0b001,
  AIR_DATA_RATE_010_2k4 = 0b010,
  AIR_DATA_RATE_011_4k8 = 0b011,
  AIR_DATA_RATE_100_9k6 = 0b100,
  AIR_DATA_RATE_101_19k2 = 0b101,
  AIR_DATA_RATE_110_38k4 = 0b110,
  AIR_DATA_RATE_111_62k5 = 0b111
};

enum LoRaSubPacketSetting {
    SPS_200_00 = 0b00,
    SPS_128_01 = 0b01,
    SPS_064_10 = 0b10,
    SPS_032_11 = 0b11
};

enum LoRaRSSIAmbientNoiseEnable {
    RSSI_AMBIENT_NOISE_ENABLED = 0b1,
    RSSI_AMBIENT_NOISE_DISABLED = 0b0
};

enum LoRaWORPeriod {
    WOR_500_000 = 0b000,
    WOR_1000_001 = 0b001,
    WOR_1500_010 = 0b010,
    WOR_2000_011 = 0b011,
    WOR_2500_100 = 0b100,
    WOR_3000_101 = 0b101,
    WOR_3500_110 = 0b110,
    WOR_4000_111 = 0b111
};

enum LoRaLBTEnable {
    LBT_ENABLED = 0b1,
    LBT_DISABLED = 0b0
};

enum LoRaRSSIEnable {
    RSSI_ENABLED = 0b1,
    RSSI_DISABLED = 0b0
};

enum LoRaFixedTransmission {
  FT_TRANSPARENT_TRANSMISSION = 0b0,
  FT_FIXED_TRANSMISSION = 0b1
};

enum LoRaTransmissionPower { // in dBm
      POWER_22 = 0b00,
      POWER_17 = 0b01,
      POWER_13 = 0b10,
      POWER_10 = 0b11
};

enum LoRaMode {
    MODE_0_NORMAL           = 0,
    MODE_1_WOR_TRANSMITTER  = 1,
    MODE_2_WOR_RECEIVER     = 2,
    MODE_3_CONFIGURATION    = 3,
    MODE_INIT               = 0xFF
};

enum LoRaCommand {
    WRITE_CFG_PWR_DWN_SAVE      = 0xC0, // Saves configuration to NON-volatile memory.
    READ_CONFIGURATION          = 0xC1,
    WRITE_CFG_PWR_DWN_LOSE      = 0xC2, // Saves configuration to volatile memory.
    WRONG_FORMAT                = 0xFF,
    RETURNED_COMMAND            = 0xC1,
    SPECIAL_WIFI_CONF_COMMAND   = 0xCF
};

enum LoRaRegAdds {
    REG_ADDS_ADDH        = 0x00,
    REG_ADDS_ADDL        = 0x01,
    REG_ADDS_SPED        = 0x02,
    REG_ADDS_TRANS_MODE  = 0x03,
    REG_ADDS_CHANNEL     = 0x04,
    REG_ADDS_OPTION      = 0x05,
    REG_ADDS_CRYPT       = 0x06,
    REG_ADDS_PID         = 0x80
};

// Enum with the payloads of the module messages.
enum LoRaPacketLength {
    // Read the configuration registers (0x00 to 0x07).
    PL_CONFIGURATION     = 0x08,
    PL_SPED              = 0x01,
    PL_OPTION            = 0x01,
    PL_TRANSMISSION_MODE = 0x01,
    PL_CHANNEL           = 0x01,
    PL_CRYPT             = 0x02,
    PL_PID               = 0x07
};

#pragma pack(push, 1)
/**
 * @brief Register 0x02 of the LoRa module. 
 */
typedef struct LoRaSpeed {
    LoRaAirDataRate   airDataRate     :3; 
    LoRaUARTParity    uartParity      :2; 
    LoRaUARTBaudrate  uartBaudRate    :3; 
} LoRaSpeed;

/**
 * @brief Register 0x03 of the LoRa module. 
 */
typedef struct LoRaTransmissionMode {
    LoRaWORPeriod           WORPeriod           :3; 
    uint8_t                 reserved2           :1; 
    LoRaLBTEnable           enableLBT           :1; 
    uint8_t                 reserved            :1; 
    LoRaFixedTransmission   fixedTransmission   :1; 
    LoRaRSSIEnable          enableRSSI          :1;
} LoRaTransmissionMode;

/**
 * @brief Register 0x05 of the LoRa module. 
 */
typedef struct LoRaOptions {
    LoRaTransmissionPower       transmissionPower   :2;
    uint8_t                     reserved            :3; 
    LoRaRSSIAmbientNoiseEnable  RSSIAmbientNoise    :1; 
    LoRaSubPacketSetting        subPacketSetting    :2; 
} LoRaOptions;

/**
 * @brief In MODE_3_PROGRAM, the inner registers of the module can be read. This struct contains the 
 * response of the module to a read request from register 0x00 to 0x07. 
 * Header + register 0x00 to 0x07. 
 */
typedef struct LoRaConfiguration {
    LoRaCommand command;
    uint8_t startAddress;
    uint8_t length;

    uint8_t addressH;
    uint8_t addressL;
    
    LoRaSpeed speed;
    LoRaOptions options;
    
    uint8_t channel;
    uint16_t getFrequency_MHz() {
        // For 868MHz LoRa E220 module, the base frequency for channel 0 is 850 MHz.
        return this->channel + 850; 
    }

    LoRaTransmissionMode transmissionMode;

    uint8_t cryptH;
    uint8_t cryptL;
} LoRaConfiguration;

/**
 * @brief Reads registers 0x80 to 0x86.
 */
typedef struct LoRaPID {
    LoRaCommand command;
    uint8_t startAddress;
    uint8_t length;

    uint8_t pid[LoRaPacketLength::PL_PID];
} LoRaPID;

#pragma pack(pop)

class LoRa {
    friend class Comms;

    public:
    /**
     * @brief Construct a new LoRa object
     * 
     * @param uart Pointer to the UART handler.
     */
    LoRa(UART* uart);

    /**
     * @brief Destroy the LoRa object
     */
    ~LoRa();

    /**
     * @brief Inits comunications with the LoRa module.
     * 
     * @return LoRaStatus
     */
    LoRaStatus init();

    /**
     * @brief Read the configuration registers of the LoRa module.
     * 
     * @param config. Where the configuration fields will be stored.
     * @return LoRaStatus 
     */
    LoRaStatus readConfigurationRegisters(LoRaConfiguration* config);

    /**
     * @brief Sets the configuration of the module.
     * 
     * @param config. Configuration fields to write.
     * @param temporarySave. Set to 1 if the configuration is to be temporarily saved. Upon reset of
     * the module, this configuration will be lost. Set to 0 to save it into non-volatile memory.
     * @return LoRaStatus. SUCCESS if the configuration was well set on the device.
     */
    LoRaStatus writeConfigurationRegisters(LoRaConfiguration config, uint8_t temporarySave = 0);

    /**
     * @brief Get the LoRa module's information. 
     * 
     * @param pid. Where the information will be stored.
     * @return LoRaStatus 
     */
    LoRaStatus getModuleInformation(LoRaPID* pid);

    /**
     * @brief Set the mode of operation of the LoRa module.
     * 
     * @param mode. The mode to be set.
     * @return LoRaStatus 
     */
    LoRaStatus setMode(LoRaMode mode);

    void auxPinISR();

    private:
    /**
     * @brief Receive data from the LoRa module.
     * 
     * @param dataLen. Number of bytes to read/receive.
     * @param dataBuffer. Where the received data will be stored.
     * @return LoRaStatus 
     */
    LoRaStatus receiveData(uint16_t dataLen, uint8_t* dataBuffer);

    /**
     * @brief Send data to the LoRa module.
     * 
     * @param dataBuffer. Where the data is stored.
     * @param dataLen. Number of bytes inside the data buffer.
     * @return LoRaStatus 
     */
    LoRaStatus writeData(uint8_t* dataBuffer, uint16_t dataLen);

    /**
     * @brief Send data to the LoRa module using DMA.
     * 
     * @param dataBuffer. Where the data is stored.
     * @param dataLen. Number of bytes inside the data buffer.
     * @return LoRaStatus 
     */
    LoRaStatus writeDataDMA(uint8_t* dataBuffer, uint16_t dataLen);

    /**
     * @brief The AUX pin is set to 0 when the device is busy. This function waits for the LoRa 
     * module to not be busy.
     * 
     * @param timeout_ms. Time to wait for the AUX pin to go back to 1.
     * @return LoRaStatus 
     */
    LoRaStatus waitAUXPin(uint32_t timeout_ms);

    /**
     * @brief Send a command to the LoRa module.
     * 
     * @param cmd. Command identifier.
     * @param addrs. Register address.
     * @param packetLength. Number of registers to operate.
     * @return uint8_t 1 if the program was well written.
     */
    uint8_t writeProgramCommand(
        LoRaCommand cmd, LoRaRegAdds addrs, LoRaPacketLength packetLength);

    private:
    UART* uart;
    LoRaMode currentMode = LoRaMode::MODE_INIT;

    CircularBuffer<uint8_t, UART_BUF_SIZE> dmaBuf;
    uint8_t currentlySendingThroughDMA = 0;

    public:
    LoRaConfiguration currentConfig;
};

#endif // LORA_h