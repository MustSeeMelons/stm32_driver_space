#ifndef INC_STM32L476XX_GPIO_DRIVER_H_
#define INC_STM32L476XX_GPIO_DRIVER_H_

#include "stm32l476xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx; // Base port to which this GPIO belongs
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

// GPIO modes
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IR_FT		4 // falling trigger
#define GPIO_MODE_IR_RT		5 // rising trigger
#define GPIO_MODE_IR_RFT	6

// GPIO output types
#define GPIO_OP_TYPE_PP 0 // push pull
#define GPIO_OP_TYPE_OD 1 // open drain

// GPIO speeds
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_FAST 	2
#define GPIO_SPEED_HIGH 	3

// GPIO pull up/down's
#define GPIO_NO_PUPD	0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2

// GPIO pin numbers
#define GPIO_PIN_N0		0
#define GPIO_PIN_N1		1
#define GPIO_PIN_N2		2
#define GPIO_PIN_N3		3
#define GPIO_PIN_N4		4
#define GPIO_PIN_N5		5
#define GPIO_PIN_N6		6
#define GPIO_PIN_N7		7
#define GPIO_PIN_N8		8
#define GPIO_PIN_N9		9
#define GPIO_PIN_N10	10
#define GPIO_PIN_N11	11
#define GPIO_PIN_N12	12
#define GPIO_PIN_N13	13
#define GPIO_PIN_N14	14
#define GPIO_PIN_N15	15

#define GPIO_AF_0   0
#define GPIO_AF_1   1
#define GPIO_AF_2   2
#define GPIO_AF_3   3
#define GPIO_AF_4   4
#define GPIO_AF_5   5
#define GPIO_AF_15  14

void GPIO_PCLK(GPIO_RegDef_t *pGPIOx, uint8_t isEnable);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandle(uint8_t pinNumber);

#endif /* INC_STM32L476XX_GPIO_DRIVER_H_ */
