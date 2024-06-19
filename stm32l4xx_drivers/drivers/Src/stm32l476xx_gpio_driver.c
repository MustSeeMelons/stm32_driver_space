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
			GPIOA_PCLK_DI()();
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
		pGPIOHandle->pGPIOx->MODER |= mode;
	} else {
		// TODO IT later
	}
	// Configure speed
	uint32_t speed = pinConfig.GPIO_PinSpeed << (2 * pinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= speed;

	// Configure pull up/down
	uint32_t pull = pinConfig.GPIO_PinPuPdControl << (2 * pinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= pull;

	// Configure otype, output type
	uint32_t o_type = pinConfig.GPIO_PinOPType << pinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER |= o_type;

	// Configure alt function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN) {

	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	return 0;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return 0;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t isEnable) {
}

void GPIO_IRQHandle(uint8_t pinNumber) {
}
