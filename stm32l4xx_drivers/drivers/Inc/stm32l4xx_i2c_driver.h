#ifndef INC_STM32L4XX_I2C_DRIVER_H_
#define INC_STM32L4XX_I2C_DRIVER_H_

#include <stdint.h>
#include "stm32l476xx.h"

typedef struct {
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress; // Slave only
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
} I2C_Handle_t;

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

void I2C_MasterSend(I2C_Handle_t *pI2CHandle, uint8_t *source, uint8_t size, uint8_t slave_addr);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_Enable(I2C_RegDef_t *pI2Cx, uint8_t isEnable);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagMask);


void I2C_AppEventCallback(I2C_Handle_t *pHandle, uint8_t event);

#endif /* INC_STM32L4XX_I2C_DRIVER_H_ */
