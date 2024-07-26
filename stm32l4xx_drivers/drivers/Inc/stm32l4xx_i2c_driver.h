#ifndef INC_STM32L4XX_I2C_DRIVER_H_
#define INC_STM32L4XX_I2C_DRIVER_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "stm32l476xx.h"

#define I2C_READY   0
#define I2C_RX      1
#define I2C_TX      2
#define I2C_DONE    3

#define I2C_EV_TX_COMPLETE 0
#define I2C_EV_RX_COMPLETE 1

#define I2C_RX_BUFFER_LENGTH 256

typedef struct {
    uint32_t I2C_SCLSpeed;
    int16_t I2C_DeviceAddress; // Slave only
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
    // Slave IT supports
    uint32_t rx_buffer[I2C_RX_BUFFER_LENGTH];
    uint16_t rx_index;

    // Master IT supports
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint8_t tx_len;
    uint8_t rx_len;
    uint8_t i2c_state;
    uint8_t addr;
    uint8_t sr; // repeated start

} I2C_Handle_t;

typedef struct {
    uint8_t repeated_start; // If 1, will not perform STOP
} I2C_Options;

// Standard mode & Fast mode
#define I2C_SCL_SPEED_SM    100000UL
#define I2C_SCL_SPEED_FM2K  200000UL
#define I2C_SCL_SPEED_FM4K  400000UL

// XXX we dont seem to be able to enable/disable acks
#define I2C_ACK_ENABLE  1
#define I2C_ACK_DISABLE 0

// XXX we seem to have much more granular control
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

void I2C_PCLK(I2C_RegDef_t *pI2Cx, uint8_t isEnable);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterWrite(I2C_Handle_t *pI2CHandle, uint8_t *source, uint8_t size, uint8_t slave_addr, I2C_Options options);
void I2C_MasterRead(I2C_Handle_t *pI2CHandle, uint8_t *destination, uint8_t size, uint8_t slave_addr, I2C_Options options);

uint8_t I2C_MasterWriteIT(I2C_Handle_t *pI2CHandle, uint8_t *source, uint8_t size, uint8_t slave_addr, I2C_Options options);
uint8_t I2C_MasterReadIT(I2C_Handle_t *pI2CHandle, uint8_t *destination, uint8_t size, uint8_t slave_addr, I2C_Options options);

void I2C_SlaveWrite(I2C_Handle_t *pI2CHandle, uint8_t data);
uint8_t I2C_SlaverRead(I2C_Handle_t *pI2CHandle);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQ_Handle(I2C_Handle_t *pI2CHandle);

void I2C_EV_IRQ_HandleIT(I2C_Handle_t *pI2CHandle);

void I2C_ER_IRQ_Handle(I2C_Handle_t *pI2CHandle);

void I2C_Enable(I2C_RegDef_t *pI2Cx, uint8_t isEnable);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagMask);

void I2C_AppEventCallback(I2C_Handle_t *pHandle, uint8_t event);

void I2C_IT_TX_Reset(I2C_Handle_t *pI2CHandle);

#endif /* INC_STM32L4XX_I2C_DRIVER_H_ */
