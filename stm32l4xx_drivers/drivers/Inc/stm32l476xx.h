#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#define __vo volatile

#include <stdint.h>

// TODO it think we missed GPIO-I

// General info
#define FLASH_BASE_ADDR 0x08000000U // 0x080FFFFF - 1023 kB, 1 mB
#define SRAM1_BASE_ADDR 0x20000000U // 0x2003FFFF - 256kB
#define SRAM2_BASE_ADDR 0x10000000U // 0x10008000 - 32kBb
#define SRAM SRAM1_BASE_ADDR
#define ROM_1 0x1FFF0000 // 0x1FFF 7000 - 28 kB
#define ROM_2 0x1FFF8000 // 0x1FFF F000 - 28 kB

// Buses
#define PERIPH_BASE 0x40000000U
#define APB1_BASE_ADDR  PERIPH_BASE
#define APB2_BASE_ADDR	0x40010000U
#define AHB1_BASE_ADDR	0x40020000U
#define AHB2_BASE_ADDR 	0x48000000U
#define AHB3_BASE_ADDR 	0xA0000000U
#define AHB4_BASE_ADDR 	0xA0001000U

// Peripherals we shall use from AHB2
#define GPIOA_BASE_ADDR (AHB2_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR (AHB2_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR (AHB2_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR (AHB2_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR (AHB2_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR (AHB2_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR (AHB2_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR (AHB2_BASE_ADDR + 0x1C00)

// Peripherals we shall use from AHB1
#define RCC_BASE_ADDR (AHB1_BASE_ADDR + 0x1000)

// Peripherals we shall use from APB1
#define I2C1_BASE_ADDR (APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR (APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR (APB1_BASE_ADDR + 0x5C00)

#define SPI2_BASE_ADDR (APB1_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR (APB1_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR (APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR (APB1_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR (APB1_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR (APB1_BASE_ADDR + 0x5000)

// Peripherals we shall use from APB2
#define USART1_BASE_ADDR (APB2_BASE_ADDR + 0x3800)
#define SPI1_BASE_ADDR (APB2_BASE_ADDR + 0x3000)
#define EXTI_BASE_ADDR (APB2_BASE_ADDR + 0x0400)

#define SYSCFG_BASE_ADDR (APB2_BASE_ADDR + 0x0000)

// Peripheral definitions, TODO document each field
typedef struct {
	volatile uint32_t MODER; 	// 0x00 offset
	volatile uint32_t OTYPER; 	// 0x04 bytes offset or a uint32, also 4 bytes, 32 bits
	volatile uint32_t OSPEEDR; 	// 0x08
	volatile uint32_t PUPDR; 	// 0x0C
	volatile uint32_t IDR;		// 0x10
	volatile uint32_t ODR;		// 0x14
	volatile uint32_t BSRR;		// 0x18
	volatile uint32_t LCKR;		// 0x1C
	// volatile uint32_t AFRL; 	// 0x20 Alternate function register low
	// volatile uint32_t AFRH; 	// 0x24 Alternate function register high
	volatile uint32_t AFR[2]; 	// [0] low, [1] high
	volatile uint32_t BRR;		// 0x28
	volatile uint32_t ASCR;		// 0x2C

} GPIO_RegDef_t;

// RCC register map
typedef struct {
	volatile uint32_t CR; // 0x00
	volatile uint32_t ICSCR; // 0x04
	volatile uint32_t CFGR; // 0x08
	volatile uint32_t PLLCFGR; // 0x0C
	volatile uint32_t PLLSAI1CFGR; // 0x10
	volatile uint32_t PLLSAI2CFGR; // 0x14
	volatile uint32_t CIER; // 0x18
	volatile uint32_t CIFR; // 0x1C
	volatile uint32_t CICR; // 0x20
	uint32_t RESERVED0;
	volatile uint32_t AHB1RSTR; // 0x28
	volatile uint32_t AHB2RSTR; // 0x2C
	volatile uint32_t AHB3RSTR; // 0x30
	uint32_t RESERVED1;
	volatile uint32_t APB1RSTR1; // 0x38
	volatile uint32_t APB1RSTR2; // 0x3C
	volatile uint32_t APB2RSTR; // 0x40
	uint32_t RESERVED2;
	volatile uint32_t AHB1ENR; // 0x48
	volatile uint32_t AHB2ENR; // 0x4c
	volatile uint32_t AHB3ENR; // 0x50
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR1; // 0x58
	volatile uint32_t APB1ENR2; // 0x5C
	volatile uint32_t APB2ENR; // 0x60
	uint32_t RESERVED4;
	volatile uint32_t AHB1SMENR; // 0x68
	volatile uint32_t AHB2SMENR; // 0x6C
	volatile uint32_t AHB3SMENR; // 0x70
	uint32_t RESERVED5;
	volatile uint32_t APB1SMENR1; // 0x78
	volatile uint32_t APB1SMENR2; // 0x7C
	volatile uint32_t APB2SMENR; // 0x80
	uint32_t RESERVED6;
	volatile uint32_t CCIPR; // 0x88
	uint32_t RESERVED7;
	volatile uint32_t BDCR; // 0x90
	volatile uint32_t CSR; // 0x94
} RCC_RegDef;

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

#define RCC ((RCC_RegDef*)RCC_BASE_ADDR)

// Clock enable macros
#define GPIOA_PCLK_EN() (RCC->AHB2ENR |= (1 << 0)) // PCLK - peripheral clock
#define GPIOB_PCLK_EN() (RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB2ENR |= (1 << 7))

#define I2C1_PCLK_EN() (RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR1 |= (1 << 23))

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR1 |= (1 << 15))

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() (RCC->APB1ENR1 |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR1 |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR1 |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR1 |= (1 << 20))

// TODO not sure about this one
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 0))

// Clock disable macros
#define GPIOA_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 7))

#define I2C1_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 23))

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 15))

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 20))

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0))

#define GPIOA_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 0)); (RCC->AHB2RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 1)); (RCC->AHB2RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 2)); (RCC->AHB2RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 3)); (RCC->AHB2RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 4)); (RCC->AHB2RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 5)); (RCC->AHB2RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 6)); (RCC->AHB2RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() do { (RCC->AHB2RSTR |= (1 << 7)); (RCC->AHB2RSTR &= ~(1 << 7)); } while(0)

// Generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#endif /* INC_STM32L476XX_H_ */
