/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32l476xx.h"
#include "stm32l476xx_gpio_driver.h"

// XXX why was this generated?
//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

void led_push_pull() {
    // Setup GPIO
    GPIO_Handle_t gpio_handle = { .pGPIOx = GPIOA, .GPIO_PinConfig = {
            .GPIO_PinNumber = GPIO_PIN_N5, .GPIO_PinMode = GPIO_MODE_OUTPUT,
            .GPIO_PinSpeed = GPIO_SPEED_FAST, .GPIO_PinPuPdControl =
                    GPIO_NO_PUPD, .GPIO_PinOPType = GPIO_OP_TYPE_PP, } };

    // Enable clock for port A
    GPIO_PCLK(GPIOA, ENABLE);

    GPIO_Init(&gpio_handle);

    while (1) {
        // Write to GPIO
        GPIO_TogglePin(GPIOA, gpio_handle.GPIO_PinConfig.GPIO_PinNumber);

        // Locking delay
        for (uint32_t i = 0; i < 500000; i++)
            ;
    }
}

// Very dim, pull up is ~40k Ohm, needs external small pull up
void led_open_drain() {
    // Setup GPIO
    GPIO_Handle_t gpio_handle = { .pGPIOx = GPIOA, .GPIO_PinConfig = {
            .GPIO_PinNumber = GPIO_PIN_N5, .GPIO_PinMode = GPIO_MODE_OUTPUT,
            .GPIO_PinSpeed = GPIO_SPEED_FAST,
            .GPIO_PinPuPdControl = GPIO_PIN_PU, .GPIO_PinOPType =
                    GPIO_OP_TYPE_OD, } };

    // Enable clock for port A
    GPIO_PCLK(GPIOA, ENABLE);

    GPIO_Init(&gpio_handle);

    while (1) {
        // Write to GPIO
        GPIO_TogglePin(GPIOA, gpio_handle.GPIO_PinConfig.GPIO_PinNumber);

        // Locking delay
        for (uint32_t i = 0; i < 500000; i++)
            ;
    }
}

// Button is on PC13
void led_button_toggle() {
    GPIO_Handle_t led_handle = {
        .pGPIOx = GPIOA,
        .GPIO_PinConfig = {
                .GPIO_PinNumber = GPIO_PIN_N5,
                .GPIO_PinMode = GPIO_MODE_OUTPUT,
                .GPIO_PinSpeed = GPIO_SPEED_FAST,
                .GPIO_PinPuPdControl = GPIO_NO_PUPD,
                .GPIO_PinOPType = GPIO_OP_TYPE_PP,
        }
    };

    GPIO_Handle_t button_handle = {
        .pGPIOx = GPIOC,
        .GPIO_PinConfig = {
                .GPIO_PinNumber = GPIO_PIN_N13,
                .GPIO_PinMode = GPIO_MODE_INPUT,
                .GPIO_PinSpeed = GPIO_SPEED_FAST,
                .GPIO_PinPuPdControl = GPIO_PIN_PU,
        }
    };

    // Enable clocks!
    GPIO_PCLK(GPIOA, ENABLE);
    GPIO_PCLK(GPIOC, ENABLE);

    // Init!
    GPIO_Init(&led_handle);
    GPIO_Init(&button_handle);

    uint8_t is_down = 0;

    while (1) {
        uint8_t button_state = GPIO_ReadFromInputPin(GPIOC, button_handle.GPIO_PinConfig.GPIO_PinNumber);

        if (button_state == 0 && is_down == 0) {
            is_down = 1;
            // Debounce
            for (uint32_t i = 0; i < 50000; i++)
                ;

            button_state = GPIO_ReadFromInputPin(GPIOC, button_handle.GPIO_PinConfig.GPIO_PinNumber);

            if (button_state == 0) {
                GPIO_TogglePin(GPIOA, led_handle.GPIO_PinConfig.GPIO_PinNumber);
            }
        } else if (button_state == 1) {
            is_down = 0;
        }
    }
}

int main(void) {
    led_button_toggle();
}
