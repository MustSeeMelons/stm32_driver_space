#include "stm32l476xx_gpio_driver.h"
#include "stm32l4xx_i2c_driver.h"
#include "string.h"

// XXX with spi PCLK seems to be 4 Mhz, we we get a max of 2 Mhz

// Arduino I2C:
// A4 - SDA
// A5 - SCL

// STM32 I2C1:
// PB6, SCL (AF4)
// PB7, SDA (AF4)

uint8_t data[] = "hodwy, partner\n";

static I2C_Handle_t handle = {
    .pI2Cx = I2C1,
    .I2C_Config = {
        .I2C_SCLSpeed = I2C_SCL_SPEED_SM,
        .I2C_DeviceAddress = 0x0,
    }
};

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
    I2C_Init(&handle);

    I2C_PCLK(handle.pI2Cx, ENABLE);

    I2C_Enable(handle.pI2Cx, ENABLE);
}

int main() {
    I2C_GPIO_Setup();

    I2C_Setup();

    uint8_t is_down = 0;

    while (1) {
        uint8_t button_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_N2);

        if (button_state == 0 && is_down == 0) {
            is_down = 1;
            delay(10000);

            I2C_MasterSend(&handle, data, (uint8_t) strlen((char*) data), 0x68);
        } else if (button_state == 1) {
            is_down = 0;
        }
    }

    return 0;
}
