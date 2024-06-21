/**
 *  Validation of GPIO PCLK, 0x4c offset of RCC.
 * 		 RCC is on AHB1 - 0x40020000. RCC offset is 0x1000
 * 		 Thus RCC is at 0x40021000.
 * 		 The bit is at 0x4002104c.
 *
 * Validation GPIO config registers:
 * 		 GPIOA is on AHB2. AHB2 is at 0x48000000. GPIOA is at offset 0x0000.
 * 		 ModeR, 0x00 offset. 10 & 11 bit: 01
 * 		 OSPEEDR, 0x08 offset, 10 & 11 bit: 10
 * 		 PUPDR, 0x0C offset, 10 & 11 bit: 00
 * 		 OTYPER, 0x04 offset, 5 bit: 0
 */