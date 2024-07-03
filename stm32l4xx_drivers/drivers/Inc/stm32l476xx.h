#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#include <stdint.h>

// XXX enums in place of these defines?

// ARM Cortex
#define NVIC_ISER0  ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1  ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2  ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3  ((volatile uint32_t*)0xE000E10C)

#define NVIC_ICER0  ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1  ((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  ((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3  ((volatile uint32_t*)0XE000E18C)

#define NVIC_PR_BASE_ADDR   ((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS 4

// General info
#define FLASH_BASE_ADDR 0x08000000U // 0x080FFFFF - 1023 kB, 1 mB
#define SRAM1_BASE_ADDR 0x20000000U // 0x2003FFFF - 256kB
#define SRAM2_BASE_ADDR 0x10000000U // 0x10008000 - 32kBb
#define SRAM SRAM1_BASE_ADDR
#define ROM_1 0x1FFF0000            // 0x1FFF 7000 - 28 kB
#define ROM_2 0x1FFF8000            // 0x1FFF F000 - 28 kB

// Buses
#define PERIPH_BASE     0x40000000U
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

// 42.6.8
typedef struct {
    volatile uint32_t CR1;      // 0x00, control register 1
    volatile uint32_t CR2;      // 0x04, control register 2
    volatile uint32_t SR;       // 0x08, status register, various flags
    volatile uint32_t DR;       // 0x0C, data register
    volatile uint32_t CRCPR;    // 0x10, CRC polynomial register
    volatile uint32_t RXCRCR;   // 0x14, CRC of received bytes
    volatile uint32_t TXCRCR;   // 0x18, CRC of transmited bytes
} SPI_RegDef_t;

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
} RCC_RegDef_t;

typedef struct {
    volatile uint32_t IMR1;     // 0x00
    volatile uint32_t EMR1;     // 0x04
    volatile uint32_t RTSR1;    // 0x08
    volatile uint32_t FTSR1;    // 0x0C
    volatile uint32_t SWIER1;   // 0x10
    volatile uint32_t PR1;      // 0x14
    uint32_t RESERVE0;
    uint32_t RESERVE1;
    uint32_t RESERVE2;
    volatile uint32_t IMR2;     // 0x20
    volatile uint32_t EMR2;     // 0x24
    volatile uint32_t RTSR2;    // 0x28
    volatile uint32_t FTSR2;    // 0x2C
    volatile uint32_t SWIER2;   // 0x30
    volatile uint32_t PR2;      // 0x34
} EXTI_RegDef_t;

typedef struct {
    volatile uint32_t MEMRMP;
    volatile uint32_t CFGR1;
    volatile uint32_t EXTICR[4];
    volatile uint32_t SCSR;
    volatile uint32_t CFGR2;
    volatile uint32_t SWPR;
    volatile uint32_t SKR;
} SYSCFG_RegDef_t;

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

#define SPI1 ((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASE_ADDR)

#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

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

#define SPI1_REG_RESET() do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET() do {(RCC->APB1RSTR1 |= (1 << 12)); (RCC->APB1RSTR1 &= ~(1 << 12));} while(0)
#define SPI3_REG_RESET() do {(RCC->APB1RSTR1 |= (1 << 12)); (RCC->APB1RSTR1 &= ~(1 << 12));} while(0)
// IRQ numbers
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40

#define NVIC_IRQ_PRI15     15

// XXX this is ugly, function with a switch would look nicer
// Returns the port code depending on address
#define GPIO_ADR_TO_CODE(x) ((x == GPIOA) ? 0 :\
                             (x == GPIOB) ? 1 :\
                             (x == GPIOC) ? 2 :\
                             (x == GPIOD) ? 3 :\
                             (x == GPIOE) ? 4 :\
                             (x == GPIOF) ? 5 :\
                             (x == GPIOG) ? 6 :\
                             (x == GPIOH) ? 7 : 0)

// Generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET      RESET
#define FLAG_SET        SET

#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_BIDIMODE    15

#define SPI_CR2_DS          8
#define SPI_CR2_SSOE        2

#define SPI_SR_RXNE     0  // Receive buffer not empty
#define SPI_SR_TXE      1  // Transmit buffer empty
#define SPI_SR_CRCE_RR  4  // CRC error flag
#define SPI_SR_MODF     5  // Mode fault
#define SPI_SR_OVR      6  // Overrun flag
#define SPI_SR_BSY      7  // Busy flag
#define SPI_SR_FRE      8  // Frame format error
#define SPI_SR_FRLVL    9  // FIFO reception level
#define SPI_SR_FTLVL    11 // FIFO transmission leve



#endif /* INC_STM32L476XX_H_ */
