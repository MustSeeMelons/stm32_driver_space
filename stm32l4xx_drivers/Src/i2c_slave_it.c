#include "stm32l476xx_gpio_driver.h"
#include "stm32l4xx_i2c_driver.h"
#include "string.h"

#define CMD_GET_LEN 0x51
#define CMD_GET_DATA 0x52

// XXX with spi PCLK seems to be 4 Mhz, we we get a max of 2 Mhz

// Arduino I2C:
// A4 - SDA
// A5 - SCL

// STM32 I2C1:
// PB6, SCL (AF4)
// PB7, SDA (AF4)

uint8_t data[256] = "this is not a test";
uint8_t slave_addr = 0x68;

static I2C_Handle_t handle = {
    .pI2Cx = I2C1,
    .I2C_Config = {
        .I2C_SCLSpeed = I2C_SCL_SPEED_SM,
        .I2C_DeviceAddress = 0x68,
    }
};

uint8_t command = 0x0;

void delay(uint32_t time) {
    for (uint32_t i = 0; i < time; i++)
        ;
}

void I2C_GPIO_Setup() {
    // White wire
    GPIO_Handle_t i2c_handle = {
        .pGPIOx = GPIOB,
        .GPIO_PinConfig = {
            .GPIO_PinMode = GPIO_MODE_ALTFN,
            .GPIO_PinSpeed = GPIO_SPEED_FAST,
            .GPIO_PinPuPdControl = GPIO_NO_PUPD, // We have external pu
            .GPIO_PinOPType = GPIO_OP_TYPE_OD,
            .GPIO_PinAltFunMode = GPIO_AF_4
        }
    };

    GPIO_PCLK(GPIOB, ENABLE);

    // SCL
    i2c_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N6;
    GPIO_Init(&i2c_handle);

    // SDA
    i2c_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N7;
    GPIO_Init(&i2c_handle);

    GPIO_Handle_t btn_handle = {
        .pGPIOx = GPIOC,
        .GPIO_PinConfig = {
            .GPIO_PinNumber = GPIO_PIN_N2,
            .GPIO_PinMode = GPIO_MODE_INPUT,
            .GPIO_PinSpeed = GPIO_SPEED_FAST,
            .GPIO_PinPuPdControl = GPIO_PIN_PU,
            .GPIO_PinOPType = GPIO_OP_TYPE_PP,
            .GPIO_PinAltFunMode = 0
        }
    };

    GPIO_PCLK(GPIOC, ENABLE);

    GPIO_Init(&btn_handle);
}

void I2C_Setup() {
    I2C_PCLK(handle.pI2Cx, ENABLE);

    I2C_Init(&handle);

    I2C_Enable(handle.pI2Cx, ENABLE);
}

void I2C_AppEventCallback(I2C_Handle_t *pHandle, uint8_t event) {

}

int main() {
    I2C_GPIO_Setup();

    // Address match IT
    handle.pI2Cx->CR1 |= (1 << I2C_ISR_ADDR);
    // Transfer buffer empty IT
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);
    // Recieve buffer full IT
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_RXIE);
    // Error IT's
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ERRIE);

    I2C_Setup();

    // XXX Setup what should be done on different events

    while (1) {

    }

    return 0;
}

void I2C1_EV_IRQHandler(void) {
    I2C_EV_IRQ_Handle(&handle);
}

void I2C1_ER_IRQHandler(void) {
    I2C_ER_IRQ_Handle(&handle);
}
