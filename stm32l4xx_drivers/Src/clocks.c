#include "stm32l476xx_gpio_driver.h"

// XXX with spi PCLK seems to be 8, we we get a max of 4 Mhz

#define RCC_CR_MSION 0
#define RCC_CR_MSIRDY 1
#define RCC_CR_HSION 8 // To enable HSI
#define RCC_CR_HSIRDY 10

#define RCC_CFGR_SW 0 // System clock switch

#define RCC_CFGR_SW_MSI 0
#define RCC_CFGR_SW_HSI 1
#define RCC_CFGR_SW_HSE 2
#define RCC_CFGR_SW_PLL 3

#define RCC_CFGR_SWS 2 // System clock switch status

#define RCC_CFGR_SWS_MSI 0
#define RCC_CFGR_SWS_HSI 1
#define RCC_CFGR_SWS_HSE 2
#define RCC_CFGR_SWS_PLL 3

#define RCC_CFGR_MCOSEL 24 // MCO type
#define MCOSEL_HSI 3
#define MCOSEL_SYSCLK 1

#define RCC_CFGR_MCOPRE 28
#define RCC_CFGR_MCOPRE_DIV1    0
#define RCC_CFGR_MCOPRE_DIV2    1
#define RCC_CFGR_MCOPRE_DIV4    2
#define RCC_CFGR_MCOPRE_DIV8    3
#define RCC_CFGR_MCOPRE_DIV16   4

int main() {

    // Enable 16 Mhz HSI and wait for it to be ready
    RCC->CR |= 1 << RCC_CR_HSION;
    while ((RCC->CR & (1 << RCC_CR_HSIRDY)) == 0)
        ;

    // Set HSI as system clock source
    RCC->CFGR |= RCC_CFGR_SW_HSI << RCC_CFGR_SW;

    while ((RCC->CFGR & (1 << RCC_CFGR_SWS)) == RCC_CFGR_SW_MSI) // != 1
        ;// Wait for HSI to be used as the system clock

    // COnfigure PA8 as MCO
    GPIO_Handle_t gpio_handle = {
            .pGPIOx = GPIOA,
            .GPIO_PinConfig = {
                    .GPIO_PinNumber = GPIO_PIN_N8,
                    .GPIO_PinMode = GPIO_MODE_ALTFN,
                    .GPIO_PinSpeed = GPIO_SPEED_HIGH,
                    .GPIO_PinPuPdControl = GPIO_NO_PUPD,
                    .GPIO_PinOPType = GPIO_OP_TYPE_PP,
                    .GPIO_PinAltFunMode = GPIO_AF_0
            }
    };

    GPIO_PCLK(GPIOA, ENABLE);
    GPIO_Init(&gpio_handle);

    // Enable MCO
    RCC->CFGR |= MCOSEL_HSI << RCC_CFGR_MCOSEL;
    // Setup MCO prescaler
    RCC->CFGR &= ~(0x7 << RCC_CFGR_MCOPRE);
    RCC->CFGR |= (RCC_CFGR_MCOPRE_DIV16 << RCC_CFGR_MCOPRE);

    while (1) {

    }

    return 0;
}
