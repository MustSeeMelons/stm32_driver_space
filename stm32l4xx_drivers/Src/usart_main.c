#include <stdint.h>
#include "stm32l476xx_gpio_driver.h"
#include "stm32l4xx_usart_driver.h"
#include "string.h"

// Arduino USART:
// D1 - TX
// D0 - RX
// [] - CTS [NONE]
// [] - RTS [NONE]

// STM32 USART1:
// PA9, TX (AF7) [Purple]
// PA10, RX (AF7) [Black]
// PA11, CTS (AF7) [NOT USED]
// PA12, RTS (AF7) [NOT USED]

static uint8_t data[] = "howdy, you mongrel";
static USART_Handle_t USARTHandle = {
    .pUSARTx = USART1,
    .USART_Config = {
        .USART_mode = USART_MODE_TXRX,
        .UASRT_Baud = USART_STD_BAUD_115200,
        .UASRT_NoOfStopBits = USART_STOPBITS_1,
        .UASRT_WordLength = USART_WORDLEN_8BITS,
        .UASRT_ParityControl = USART_PARITY_DISABLE,
        .UASRT_HwFlowControl = USART_HW_FLOW_CTRL_NONE
    }
};

void delay(uint32_t time) {
    for (uint32_t i = 0; i < time; i++)
        ;
}

void GPIO_Setup() {
    GPIO_Handle_t gpio_uart = {
            .pGPIOx = GPIOA,
            .GPIO_PinConfig = {
                    .GPIO_PinMode = GPIO_MODE_ALTFN,
                    .GPIO_PinSpeed = GPIO_SPEED_FAST,
                    .GPIO_PinPuPdControl = GPIO_NO_PUPD,
                    .GPIO_PinOPType = GPIO_OP_TYPE_PP,
                    .GPIO_PinAltFunMode = GPIO_AF_7
            }
    };

    GPIO_PCLK(gpio_uart.pGPIOx, ENABLE);

    // TX
    gpio_uart.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N9;
    GPIO_Init(&gpio_uart);

    // RX
    gpio_uart.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N10;
    GPIO_Init(&gpio_uart);

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

    GPIO_PCLK(btn_handle.pGPIOx, ENABLE);

    GPIO_Init(&btn_handle);
}

void USART_Setup() {
    USART_PCLK(USARTHandle.pUSARTx, ENABLE);

    USART_Init(&USARTHandle);

    USART_Enable(USARTHandle.pUSARTx, ENABLE);
}

// 115200
// 1 stop bit
// 8 bits
// no parity

int main() {
    GPIO_Setup();

    USART_Setup();

    uint8_t is_down = 0;

    while (1) {
        uint8_t button_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_N2);

        if (button_state == 0 && is_down == 0) {
            is_down = 1;
            delay(50000);

            USART_Write(&USARTHandle, data, strlen((char*)data));
        } else if (button_state == 1 && is_down != 0) {
            is_down = 0;
        }
    }

    return 0;
}
