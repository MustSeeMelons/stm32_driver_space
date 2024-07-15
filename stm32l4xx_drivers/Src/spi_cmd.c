#include <stdint.h>
#include <string.h>
#include "stm32l476xx.h"
#include "stm32l476xx_gpio_driver.h"
#include "stm32l476xx_spi_driver.h"

#define CMD_LED_CTRL    0x50
#define CMD_SENSOR_READ 0x51
#define CMD_LED_READ    0x52
#define CMD_PRINT       0x53
#define CMD_ID_READ     0x54

#define LED_ON  1
#define LED_OFF 0

#define ANALOG_PIN0 0
#define ANALOG_PIN1 1
#define ANALOG_PIN2 2
#define ANALOG_PIN3 3
#define ANALOG_PIN4 4

#define LED_PIN 9

// SPI2_SCK PB13
// SPI2_MISO PB14
// SPI2 MOSI PB15
// SPI2 CS PB6

// Uno pins:
// SS   10
// MOSI 11
// MISO 12
// SCK  13

static uint8_t dummy_byte = 0xff; // To send for when we want to "ask" something
static uint8_t dummy_data = 0;

void delay(uint32_t time) {
    for (uint32_t i = 0; i < time; i++)
        ;
}

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
    spi_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N14;
    GPIO_Init(&spi_handle);

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

uint8_t SPI_VerifyResponse(uint8_t ack) {
    return 1;

    if (ack == 0xF5) {
        return 1;
    } else {
        return 0;
    }
}

void process_led_toggle(SPI_Handle_t *spi_handle) {
    // Send command
    uint8_t cmd_code = CMD_LED_CTRL;
    SPI_SendData(spi_handle->pSPIx, &cmd_code, 1);
    // Clear RXNE
    SPI_ReceiveData(spi_handle->pSPIx, &dummy_data, 1);

    delay(1000);

    // Get ACK/NACK
    uint8_t ack;
    uint8_t args[2];

    SPI_SendData(spi_handle->pSPIx, &dummy_byte, 1);
    SPI_ReceiveData(spi_handle->pSPIx, &ack, 1);

    if (SPI_VerifyResponse(ack)) {
        args[0] = LED_PIN;
        args[1] = LED_ON;

        SPI_SendData(spi_handle->pSPIx, args, 2);
    }

    // 1 for busy, Wait for busy flag to clear
    while (spi_handle->pSPIx->SR & (0x1 << SPI_SR_BSY))
        ;
}

void process_sensor_read(SPI_Handle_t *spi_handle) {
    uint8_t cmd_code = CMD_SENSOR_READ;
    SPI_SendData(spi_handle->pSPIx, &cmd_code, 1);

    delay(1000);

    // Clear RXNE
    SPI_ReceiveData(spi_handle->pSPIx, &dummy_data, 1);

    // Get ACK/NACK
    uint8_t ack;
    uint8_t args[2];

    SPI_SendData(spi_handle->pSPIx, &dummy_byte, 1);
    SPI_ReceiveData(spi_handle->pSPIx, &ack, 1);

    if (SPI_VerifyResponse(ack)) {
        args[0] = ANALOG_PIN0;

        SPI_SendData(spi_handle->pSPIx, args, 1);

        // Clear RXNE
        SPI_ReceiveData(spi_handle->pSPIx, &dummy_data, 1);

        // Delay, so slave is ready with data
        delay(100);

        // Get data from slave
        SPI_SendData(spi_handle->pSPIx, &dummy_byte, 1);

        uint8_t analog_read;
        SPI_ReceiveData(spi_handle->pSPIx, &analog_read, 1);
    }

    // 1 for busy, Wait for busy flag to clear
    while (spi_handle->pSPIx->SR & (0x1 << SPI_SR_BSY))
        ;
}

void process_led_read(SPI_Handle_t *spi_handle) {
    uint8_t cmd_code = CMD_LED_READ;
    SPI_SendData(spi_handle->pSPIx, &cmd_code, 1);

    delay(1000);

    // Clear RXNE
    SPI_ReceiveData(spi_handle->pSPIx, &dummy_data, 1);

    // Get ACK/NACK
    uint8_t ack;
    uint8_t args[2];

    SPI_SendData(spi_handle->pSPIx, &dummy_byte, 1);
    SPI_ReceiveData(spi_handle->pSPIx, &ack, 1);

    if (SPI_VerifyResponse(ack)) {
        args[0] = LED_PIN;

        SPI_SendData(spi_handle->pSPIx, args, 1);

        // Clear RXNE
        SPI_ReceiveData(spi_handle->pSPIx, &dummy_data, 1);

        // Delay, so slave is ready with data
        delay(100);

        // Get data from slave
        SPI_SendData(spi_handle->pSPIx, &dummy_byte, 1);

        uint8_t led_state;
        SPI_ReceiveData(spi_handle->pSPIx, &led_state, 1);
    }
}

void process_print(SPI_Handle_t *spi_handle){}

void process_id_read(SPI_Handle_t *spi_handle){
    // Send command
    // TODO we really should take it as a param and make the initial "handshake" a function
    uint8_t cmd = CMD_ID_READ;
    SPI_SendData(spi_handle->pSPIx, &cmd, 1);

    delay(1000);

    // Reset RX flag
    SPI_ReceiveData(spi_handle->pSPIx, &dummy_data, 1);

    delay(1000);

    // Send dummy data to obtain ack
    uint8_t ack;
    SPI_SendData(spi_handle->pSPIx, &dummy_byte, 1);
    SPI_ReceiveData(spi_handle->pSPIx, &ack, 1);

    const uint8_t buff_size = 10;

    if (SPI_VerifyResponse(ack)) {
        uint8_t buffer[buff_size];

        for (uint32_t i = 0; i < buff_size; i++) {
            // Send over dummy bytes to read id bytes
            SPI_SendData(spi_handle->pSPIx, &dummy_byte, 1);
            uint8_t byte;
            SPI_ReceiveData(spi_handle->pSPIx, &byte, 1);
            buffer[i] = byte;

        }
    }
}

void spi_send() {
    uint8_t active_cmd = CMD_ID_READ;

    SPI_GPIO_Setup();

    SPI_Handle_t spi_handle = {
        .pSPIx = SPI2,
        .SPIConfig = {
            .SPI_DeviceMode = SPI_DEVICE_MODE_MASTER,
            .SPI_BusConfig = SPI_BUS_CONFIG_FD,
            .SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16, // 2 Mhz
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
            delay(50000);
            SPI_Enable(spi_handle.pSPIx, ENABLE);

            switch (active_cmd) {
                case 0x50:
                    process_led_toggle(&spi_handle);
                    active_cmd = CMD_SENSOR_READ;
                    break;
                case 0x51:
                    process_sensor_read(&spi_handle);
                    active_cmd = CMD_LED_READ;
                    break;
                case 0x52:
                    process_led_read(&spi_handle);
                    active_cmd = CMD_ID_READ;
                    break;
                case 0x53:
                    process_print(&spi_handle);
                    break;
                case 0x54:
                    process_id_read(&spi_handle);
                    active_cmd = CMD_LED_CTRL;
                    break;
                default:
                    break;
            }

            SPI_Enable(spi_handle.pSPIx, DISABLE);
        } else if (button_state == 1) {
            is_down = 0;
        }
    }
}

int main(void) {
    spi_send();
}
