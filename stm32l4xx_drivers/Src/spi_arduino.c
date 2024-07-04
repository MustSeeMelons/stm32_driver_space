#include <stdint.h>
#include <string.h>
#include "stm32l476xx.h"
#include "stm32l476xx_gpio_driver.h"
#include "stm32l476xx_spi_driver.h"

void delay(uint32_t time) {
    for (uint32_t i = 0; i < time; i++)
        ;
}

// SPI2_SCK PB13
// SPI2_MISO PB14
// SPI2 MOSI PB15
// SPI2 CS PB6

// Uno pins:
// SS   10
// MOSI 11
// MISO 12
// SCK  13

void SPI_GPIO_Setup() {
    // White wire
    GPIO_Handle_t spi_handle = {
        .pGPIOx = GPIOB,
        .GPIO_PinConfig = {
            .GPIO_PinMode = GPIO_MODE_ALTFN,
            .GPIO_PinSpeed = GPIO_SPEED_FAST,
            .GPIO_PinPuPdControl = GPIO_NO_PUPD,
            .GPIO_PinOPType = GPIO_OP_TYPE_PP,
            .GPIO_PinAltFunMode = GPIO_AF_5
        }
    };

    GPIO_PCLK(GPIOB, ENABLE);

    // MOSI
    spi_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N15;
    GPIO_Init(&spi_handle);

    // SCK
    spi_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N13;
    GPIO_Init(&spi_handle);

    // MISO
    // handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N14;
    // GPIO_Init(&handle);

    // NSS
    spi_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N12;
    GPIO_Init(&spi_handle);

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

void spi_send() {
    char msg[] = "howdy partner";

    SPI_GPIO_Setup();

    SPI_Handle_t spi_handle = {
        .pSPIx = SPI2,
        .SPIConfig = {
            .SPI_DeviceMode = SPI_DEVICE_MODE_MASTER,
            .SPI_BusConfig = SPI_BUS_CONFIG_FD,
            .SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4, // 2 Mhz
            .SPI_DFF = SPI_DFF_8BITS,
            .SPI_CPOL = SPI_CPOL_LOW,
            .SPI_CPHA = SPI_CPAH_LOW,
            .SPI_SSM = SPI_SSM_DI
        }
    };

    SPI_Init(&spi_handle);

    SPI_SSOEConfig(spi_handle.pSPIx, ENABLE);

    uint8_t is_down = 0;

    while (1) {
        uint8_t button_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_N2);

        if (button_state == 0 && is_down == 0) {
            is_down = 1;
            delay(10000);
            SPI_Enable(spi_handle.pSPIx, ENABLE);

            uint8_t str_len = strlen(msg);

            SPI_SendData(spi_handle.pSPIx, &str_len, 1);

            SPI_SendData(spi_handle.pSPIx, (uint8_t*) msg, str_len);

            // 1 for busy, Wait for busy flag to clear
            while (spi_handle.pSPIx->SR & (0x1 << SPI_SR_BSY))
                ;

            SPI_Enable(spi_handle.pSPIx, DISABLE);
        } else if (button_state == 1) {
            is_down = 0;
        }
    }
}

int main(void) {
    spi_send();
}
