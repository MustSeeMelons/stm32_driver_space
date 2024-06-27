Validation of GPIO PCLK, 0x4c offset of RCC.
	 RCC is on AHB1 - 0x40020000. RCC offset is 0x1000
	 Thus RCC is at 0x40021000.
	 The bit is at 0x4002104c.

Validation GPIO config registers:
	 GPIOA is on AHB2. AHB2 is at 0x48000000. GPIOA is at offset 0x0000.
	 ModeR, 0x00 offset. 10 & 11 bit: 01
	 OSPEEDR, 0x08 offset, 10 & 11 bit: 10
	 PUPDR, 0x0C offset, 10 & 11 bit: 00
	 OTYPER, 0x04 offset, 5 bit: 0
 

Validation of GPIO interrupt:
	Setup EXTI: (OK)
		EXTI1, on APB2(0x4001 0000) 0x4001 0400
			FTSR1, 0x0C: FT2 or bit 2
			RTSR1, 0x08: all 0s
	
	SYSCFG PCLK (OK)
		RCC is on AHB1 - 0x40020000. RCC offset is 0x1000
		Thus RCC is at 0x40021000.
		ABB2ENR is at 0x60 => 0x40021060
		RCC_APB2ENR, 0th bit 1 for clock.
	
	Setup SYSCFG EXTICRs: (OK)
		APB2(0x4001 0000) , SYSCFG 0x4001 0000
			SYSCFG EXTICR 0x08, 0x0C, 0x10, 0x14, we need EXTI1, thus 0x08, bits 8-11 [0011]
	
	More EXTI: (OK)
		EXTI1, on APB2(0x4001 0000) 0x4001 0400
		IMR1 offset 0x00: 2nd bit 1, 24-31 are 1
		
	NVIC, IRQNum = 8, NVIC_ISER0, base 0xE000E100, 8th bit is 1 (OK)
	NVIC, Priority. base 0xE000E400. Each Interupt has 8 bits. (OK)
		We need 8th for PD2, so 9th "index", eg 0x08 offset, firt byte.