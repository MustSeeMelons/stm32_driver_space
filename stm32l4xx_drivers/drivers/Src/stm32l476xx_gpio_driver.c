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

	// non IT
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Mode takes 2 bits, all go consecutively so we can do 2 * pin number
		mode = pinConfig.GPIO_PinMode << (2 * pinConfig.GPIO_PinNumber);

		// Must clear before set!
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (pinConfig.GPIO_PinNumber * 2));
		pGPIOHandle->pGPIOx->MODER |= mode;
	} else {
		// TODO IT later
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

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t isEnable) {
}

void GPIO_IRQHandle(uint8_t pinNumber) {
}
