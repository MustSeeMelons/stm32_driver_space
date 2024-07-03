#ifndef INC_STM32L476XX_SPI_DRIVER_H_
#define INC_STM32L476XX_SPI_DRIVER_H_

#include "stm32l476xx.h"

typedef struct {
    uint8_t SPI_DeviceMode; // Master/Slave
    uint8_t SPI_BusConfig;  // Simplex, Duplex, Half
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;        // Data frame format
    uint8_t SPI_CPOL;       // Clock polarity
    uint8_t SPI_CPHA;       // Clock phase
    uint8_t SPI_SSM;        // Software slave select
} SPI_Config_t;

typedef struct {
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
} SPI_Handle_t;

#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE   0

#define SPI_BUS_CONFIG_FD       1
#define SPI_BUS_CONFIG_HD       2
#define SPI_BUS_CONFIG_S_RXONLY 3

#define SPI_SCLK_SPEED_DIV2     0
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7

// Data frame foramt
#define SPI_DFF_8BITS   0 // Default
#define SPI_DFF_16_BITS 1

#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW  0

#define SPI_CPAH_HIGH 1
#define SPI_CPAH_LOW  0

#define SPI_SSM_EN  1
#define SPI_SSM_DI  0

#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG (1 << SPI_SR_BSY)

void SPI_PCLK(SPI_RegDef_t *pSPIx, uint8_t isEnable);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *source, uint32_t size);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *destination, uint32_t size);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandle(SPI_Handle_t *pHandle);

void SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t isEnable);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t isEnable);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t isEnable);

#endif /* INC_STM32L476XX_SPI_DRIVER_H_ */
