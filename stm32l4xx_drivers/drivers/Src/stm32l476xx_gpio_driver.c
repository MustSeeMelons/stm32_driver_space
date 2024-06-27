#include "stm32l476xx_gpio_driver.h"

void GPIO_PCLK(GPIO_RegDef_t *pGPIOx, uint8_t isEnable) {
	if (isEnable == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	// Configure mode
	GPIO_PinConfig_t pinConfig = pGPIOHandle->GPIO_PinConfig;
	uint32_t mode = 0;

	uint8_t pin_mode = pinConfig.GPIO_PinMode;
	uint8_t pin_number = pinConfig.GPIO_PinNumber;

	// non IT
	if (pin_mode <= GPIO_MODE_ANALOG) {
		// Mode takes 2 bits, all go consecutively so we can do 2 * pin number
		mode = pinConfig.GPIO_PinMode << (2 * pinConfig.GPIO_PinNumber);

		// Must clear before set!
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (pinConfig.GPIO_PinNumber * 2));
		pGPIOHandle->pGPIOx->MODER |= mode;
	} else {
	    // On reset mode will be analog, mode cant be analog for IT to work
        pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (pinConfig.GPIO_PinNumber * 2));
        pGPIOHandle->pGPIOx->MODER |= GPIO_MODE_INPUT;

	    // PA0, PB0 etc are connected to EXTI0
	    // PA1, PB1 etc are connected to EXTI1
	    // ...
	    // PA15, PB15 are connected to EXTI15
	    // Lines 0-15 are for GPIO's

	    volatile uint32_t *FTSR = &EXTI->FTSR1;
	    volatile uint32_t *RTSR = &EXTI->RTSR1;
	    volatile uint32_t *IMR = &EXTI->IMR1;
	    // GPIO => EXTI(Edge detection, interupt delivery) => NVIC(Enable and configure IRQ) => CPU
	    // IRQ - interupt request
	    // NVIC - nested vectored interupt controler
        if (pin_mode == GPIO_MODE_IR_FT) {
            // Configure falling edge control register (FTSR)
            *FTSR |= (1 << pin_number);
            *RTSR &= ~(1 << pin_number); // Reset RTSR just in case
        } else if (pin_mode == GPIO_MODE_IR_RT) {
            // Configure rising edge control register (RTSR)
            *RTSR |= (1 << pin_number);
            *FTSR &= ~(1 << pin_number); // Reset FTSR just in case
        } else if (pin_mode == GPIO_MODE_IR_RFT) {
            // Configure both registers
            *RTSR |= (1 << pin_number);
            *FTSR |= (1 << pin_number);
        }

        // Configure GPIO port in SYSCFG_EXTIR
        uint8_t index = pin_number / 4;
        uint8_t offset = pin_number % 4;

        uint8_t portcode = GPIO_ADR_TO_CODE(pGPIOHandle->pGPIOx);

        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[index] &= ~(portcode << (offset * 4));
        SYSCFG->EXTICR[index] |= portcode << (offset * 4);

        // Enable EXTI interupt delivery using IMR (Interupt mask register)
        *IMR |= (1 << pin_number);
	}
	// Configure speed
	uint32_t speed = pinConfig.GPIO_PinSpeed << (2 * pinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (pinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIOx->OSPEEDR |= speed;

	// Configure pull up/down
	uint32_t pull = pinConfig.GPIO_PinPuPdControl << (2 * pinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (pinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIOx->PUPDR |= pull;

	// Configure otype, output type
	uint32_t o_type = pinConfig.GPIO_PinOPType << pinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER |= o_type;

	// Configure alt function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {

		uint32_t index = pinConfig.GPIO_PinNumber / 8;
		uint32_t offset = pinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[index] &= ~(0xF << (4 * offset));
		pGPIOHandle->pGPIOx->AFR[index] |= pinConfig.GPIO_PinAltFunMode << (4 * offset);
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	return (uint8_t) (pGPIOx->IDR >> pinNumber) & 0x00000001;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t) pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= 1 << pinNumber;
	} else if (value == GPIO_PIN_RESET) {
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR |= value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << pinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable) {
    if (isEnable == ENABLE) {
        if (IRQNumber <= 31) {
            // ISER0
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 32 && IRQNumber < 64) {
            // ISER1
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // ISER2
            *NVIC_ISER2 |= (1 << IRQNumber % 32);
        }
    } else {
        if (IRQNumber <= 31) {
            // ICER0
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 32 && IRQNumber < 64) {
            // ICER1
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // ICER2
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    uint8_t index = IRQNumber / 4;
    uint8_t offset = IRQNumber % 4;

    // PR lower 4 bits are ignored, thus have to shift prio by 4
    IRQPriority = IRQPriority << NO_PR_BITS;

    *(NVIC_PR_BASE_ADDR + index) |= IRQPriority << (offset * 8);
}

void GPIO_IRQHandle(uint8_t pinNumber) {
    // clear the EXTI PR (pending) register
    if (EXTI->PR1 & (1 << pinNumber)) {
        EXTI->PR1 |= (1 << pinNumber);
    }
}
