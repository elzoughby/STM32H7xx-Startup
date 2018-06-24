/*============================================================================
 Name        : main.c
 Author      : Ahmed Elzoughby
 Version     : 1.0.0
 Copyright   : Copyright (c) 2018 Ahmed Elzoughby
 Description : A demonstration of the independent startup code for STM32H7
 Note        : Tested on Nucleo-H743ZI development board
=============================================================================*/



/*======================= Includes ========================*/
#include <stdint.h>


/*==================== Private macros =====================*/
#define RCC_AHB4ENR			(*((volatile uint32_t *) 0x580244E0))
#define RCC_APB4ENR			(*((volatile uint32_t *) 0x580244F4))
#define SYSCFG_EXTICR4		(*((volatile uint32_t *) 0x58000414))
#define EXTI_CPUIMR1		(*((volatile uint32_t *) 0x58000080))
#define EXTI_RTSR1			(*((volatile uint32_t *) 0x58000000))
#define EXTI_CPUPR1			(*((volatile uint32_t *) 0x58000088))
#define NVIC_ISER1			(*((volatile uint32_t *) 0xE000E104))
#define GPIO_PORTB_MODER	(*((volatile uint32_t *) 0x58020400))
#define GPIO_PORTB_OTYPER	(*((volatile uint32_t *) 0x58020404))
#define GPIO_PORTB_OSPEEDR	(*((volatile uint32_t *) 0x58020408))
#define GPIO_PORTB_ODR		(*((volatile uint32_t *) 0x58020414))
#define GPIO_PORTC_MODER	(*((volatile uint32_t *) 0x58020800))
#define GPIO_PORTC_PUPDR	(*((volatile uint32_t *) 0x5802080C))
#define GPIO_PORTC_IDR		(*((volatile uint32_t *) 0x58020810))

#define EXTI13_IRQn			40


/*=================== Global variables ====================*/
volatile uint8_t flashing = 0;


/*================== function prototypes ==================*/
void delayCountingTo(uint32_t count);
void EXTI15_10_IRQHandler(void);


/*==================== main function ======================*/
int main(void) {

	// enable clock for system configuration
	RCC_APB4ENR |= (1 << 1);
	// enable clock for GPIO PORTB on AHB bus
	RCC_AHB4ENR |= (1 << 1);
	// enable clock for GPIO PORTC on AHB bus
	RCC_AHB4ENR |= (1 << 2);

	// set Green LED mode register with 01 for General purpose output mode
	GPIO_PORTB_MODER |= (1 << 0);
	GPIO_PORTB_MODER &= ~(1 << 1);
	// set User Button pin mode with 00 for input
	GPIO_PORTC_MODER &= ~(1 << 26);
	GPIO_PORTC_MODER &= ~(1 << 27);

	// set Green LED output type as push-pull
	GPIO_PORTB_OTYPER &= ~(1 << 0);
	// enable pull-down resistor for User Button pin
	GPIO_PORTC_PUPDR &= ~(1 << 26);
	GPIO_PORTC_PUPDR |= (1 << 27);

	// enable External Interrupt #13
	EXTI_CPUIMR1 |= (1 << 13);
	// set interrupt on rising edge
	EXTI_RTSR1 |= (1 << 13);
	// enable interrupt for User Button pin PC13
	SYSCFG_EXTICR4 &= ~(0xD << 4);
	SYSCFG_EXTICR4 |= (2 << 4);
	// enable EXTI interrupt NVIC IRQ
	NVIC_ISER1 |= (1 << (EXTI13_IRQn - 32));

	while (1) {

		if (flashing) {
			/* turn on Green LED */
			GPIO_PORTB_ODR |= (1 << 0);

			delayCountingTo(1000000);

			/* turn off Green LED */
			GPIO_PORTB_ODR &= ~(1 << 0);

			delayCountingTo(1000000);
		} else {
			/* turn off Green LED */
			GPIO_PORTB_ODR &= ~(1 << 0);
		}
	}

	return 0;
}


/*================= Function definitions ==================*/
void delayCountingTo(uint32_t count) {

	volatile uint32_t i;
	for (i = 0; i < count; i++);
}

void EXTI15_10_IRQHandler(void) {

	/* if EXTI13 occurred */
	if(EXTI_CPUPR1 & (1 << 13)) {
		/* toggle flashing */
		flashing = flashing ? 0 : 1;
		/* clear EXTI13 status bit */
		EXTI_CPUPR1 |= (1 << 13);
	}
}

