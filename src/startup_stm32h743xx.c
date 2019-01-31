/*==============================================================================
 Name        : startup_stm32h743xx.c
 Version     : 1.0.0
 Description : An independent startup code for STM32H7 MCUs written in C
--------------------------------------------------------------------------------

 The MIT License (MIT)
 Copyright (c) 2018 Ahmed Elzoughby

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
===============================================================================*/



/*======================= Includes ========================*/
#include <stdint.h>


/*===== define the Reset Clock Control registers bank =====*/
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t ICSCR;
	volatile uint32_t CRRCR;
	uint32_t RESERVED0;
	volatile uint32_t CFGR;
	uint32_t RESERVED1;
	volatile uint32_t D1CFGR;
	volatile uint32_t D2CFGR;
	volatile uint32_t D3CFGR;
	uint32_t RESERVED2;
	volatile uint32_t PLLCKSELR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t PLL1DIVR;
	volatile uint32_t PLL1FRACR;
	volatile uint32_t PLL2DIVR;
	volatile uint32_t PLL2FRACR;
	volatile uint32_t PLL3DIVR;
	volatile uint32_t PLL3FRACR;
	uint32_t RESERVED3;
	volatile uint32_t D1CCIPR;
	volatile uint32_t D2CCIP1R;
	volatile uint32_t D2CCIP2R;
	volatile uint32_t D3CCIPR;
	uint32_t RESERVED4;
	volatile uint32_t CIER;
	volatile uint32_t CIFR;
	volatile uint32_t CICR;
	uint32_t RESERVED5;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB4RSTR;
	volatile uint32_t APB3RSTR;
	volatile uint32_t APB1LRSTR;
	volatile uint32_t APB1HRSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB4RSTR;
	volatile uint32_t GCR;
	uint32_t RESERVED7;
	volatile uint32_t D3AMR;
	uint32_t RESERVED8[9];
	volatile uint32_t RSR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB4ENR;
	volatile uint32_t APB3ENR;
	volatile uint32_t APB1LENR;
	volatile uint32_t APB1HENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB4ENR;
	uint32_t RESERVED9;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB4LPENR;
	volatile uint32_t APB3LPENR;
	volatile uint32_t APB1LLPENR;
	volatile uint32_t APB1HLPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t APB4LPENR;
	uint32_t RESERVED10[4];
} rccStruct;


/*==================== Private macros =====================*/
#define HWREG(x) (*((volatile uint32_t *)(x)))
#define RCC_BASE	0x58024400
#define RCC			((rccStruct*) RCC_BASE)


/*================== function prototypes ==================*/
extern int main();
static void SystemInit();
extern void __libc_init_array();

void Default_Handler()              __attribute__((weak));
void Reset_Handler()                __attribute__((weak));
void NMI_Handler()                  __attribute__((weak));
void HardFault_Handler()            __attribute__((weak));
void MemManage_Handler()            __attribute__((weak));
void BusFault_Handler()             __attribute__((weak));
void UsageFault_Handler()           __attribute__((weak));
void SVC_Handler()                  __attribute__((weak));
void DebugMon_Handler()             __attribute__((weak));
void PendSV_Handler()               __attribute__((weak));
void SysTick_Handler()              __attribute__((weak));

void WWDG_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void PVD_AVD_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void ADC_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void FDCAN1_IT0_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));
void FDCAN2_IT0_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));
void FDCAN1_IT1_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));
void FDCAN2_IT1_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler()         __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler()         __attribute__ ((weak, alias("Default_Handler")));
void TIM8_BRK_TIM12_IRQHandler()    __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler()__attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void FMC_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void SDMMC1_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void ETH_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
void FDCAN_CAL_IRQHandler()         __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void USART6_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler()    __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_WKUP_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void DCMI_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void RNG_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void UART7_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void UART8_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void SPI4_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void SPI5_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void SPI6_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void SAI1_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void LTDC_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void LTDC_ER_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void DMA2D_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void SAI2_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void QUADSPI_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void CEC_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void I2C4_EV_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void I2C4_ER_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void SPDIF_RX_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_EP1_OUT_IRQHandler()    __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_EP1_IN_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void DMAMUX1_OVR_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_Master_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMA_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMB_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMC_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIMD_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_TIME_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void HRTIM1_FLT_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT0_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT1_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT2_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void DFSDM1_FLT3_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void SAI3_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void SWPMI1_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void TIM15_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void TIM16_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void TIM17_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void MDIOS_WKUP_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));
void MDIOS_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void JPEG_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void MDMA_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void SDMMC2_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void HSEM1_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void ADC3_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void DMAMUX2_OVR_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel0_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel1_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel2_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel3_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel4_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel5_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel6_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void BDMA_Channel7_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void COMP1_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
void LPTIM2_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void LPTIM3_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void LPTIM4_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void LPTIM5_IRQHandler()            __attribute__ ((weak, alias("Default_Handler")));
void LPUART1_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
void CRS_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
void SAI4_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
void WAKEUP_PIN_IRQHandler()        __attribute__ ((weak, alias("Default_Handler")));


/*=================== Global variables ====================*/
/* Highest address of the user mode stack */
extern const volatile uint32_t _estack;
/* start address for the initialization values of the .data section.
defined in linker script */
extern uint32_t _sidata;
/* start address for the .data section. defined in linker script */
extern uint32_t _sdata;
/* end address for the .data section. defined in linker script */
extern uint32_t _edata;
/* start address for the .bss section. defined in linker script */
extern uint32_t _sbss;
/* end address for the .bss section. defined in linker script */
extern uint32_t _ebss;

/* The Interrupt Vector Table */
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
		((void (*)(void)) (&_estack)),
		Reset_Handler,
		NMI_Handler,
		HardFault_Handler,
		MemManage_Handler,
		BusFault_Handler,
		UsageFault_Handler,
		0,
		0,
		0,
		0,
		SVC_Handler,
		DebugMon_Handler,
		0,
		PendSV_Handler,
		SysTick_Handler,

		/* External Interrupts */
		WWDG_IRQHandler,
		PVD_AVD_IRQHandler,
		TAMP_STAMP_IRQHandler,
		RTC_WKUP_IRQHandler,
		FLASH_IRQHandler,
		RCC_IRQHandler,
		EXTI0_IRQHandler,
		EXTI1_IRQHandler,
		EXTI2_IRQHandler,
		EXTI3_IRQHandler,
		EXTI4_IRQHandler,
		DMA1_Stream0_IRQHandler,
		DMA1_Stream1_IRQHandler,
		DMA1_Stream2_IRQHandler,
		DMA1_Stream3_IRQHandler,
		DMA1_Stream4_IRQHandler,
		DMA1_Stream5_IRQHandler,
		DMA1_Stream6_IRQHandler,
		ADC_IRQHandler,
		FDCAN1_IT0_IRQHandler,
		FDCAN2_IT0_IRQHandler,
		FDCAN1_IT1_IRQHandler,
		FDCAN2_IT1_IRQHandler,
		EXTI9_5_IRQHandler,
		TIM1_BRK_IRQHandler,
		TIM1_UP_IRQHandler,
		TIM1_TRG_COM_IRQHandler,
		TIM1_CC_IRQHandler,
		TIM2_IRQHandler,
		TIM3_IRQHandler,
		TIM4_IRQHandler,
		I2C1_EV_IRQHandler,
		I2C1_ER_IRQHandler,
		I2C2_EV_IRQHandler,
		I2C2_ER_IRQHandler,
		SPI1_IRQHandler,
		SPI2_IRQHandler,
		USART1_IRQHandler,
		USART2_IRQHandler,
		USART3_IRQHandler,
		EXTI15_10_IRQHandler,
		RTC_Alarm_IRQHandler,
		0,
		TIM8_BRK_TIM12_IRQHandler,
		TIM8_UP_TIM13_IRQHandler,
		TIM8_TRG_COM_TIM14_IRQHandler,
		TIM8_CC_IRQHandler,
		DMA1_Stream7_IRQHandler,
		FMC_IRQHandler,
		SDMMC1_IRQHandler,
		TIM5_IRQHandler,
		SPI3_IRQHandler,
		UART4_IRQHandler,
		UART5_IRQHandler,
		TIM6_DAC_IRQHandler,
		TIM7_IRQHandler,
		DMA2_Stream0_IRQHandler,
		DMA2_Stream1_IRQHandler,
		DMA2_Stream2_IRQHandler,
		DMA2_Stream3_IRQHandler,
		DMA2_Stream4_IRQHandler,
		ETH_IRQHandler,
		ETH_WKUP_IRQHandler,
		FDCAN_CAL_IRQHandler,
		0,
		0,
		0,
		0,
		DMA2_Stream5_IRQHandler,
		DMA2_Stream6_IRQHandler,
		DMA2_Stream7_IRQHandler,
		USART6_IRQHandler,
		I2C3_EV_IRQHandler,
		I2C3_ER_IRQHandler,
		OTG_HS_EP1_OUT_IRQHandler,
		OTG_HS_EP1_IN_IRQHandler,
		OTG_HS_WKUP_IRQHandler,
		OTG_HS_IRQHandler,
		DCMI_IRQHandler,
		0,
		RNG_IRQHandler,
		FPU_IRQHandler,
		UART7_IRQHandler,
		UART8_IRQHandler,
		SPI4_IRQHandler,
		SPI5_IRQHandler,
		SPI6_IRQHandler,
		SAI1_IRQHandler,
		LTDC_IRQHandler,
		LTDC_ER_IRQHandler,
		DMA2D_IRQHandler,
		SAI2_IRQHandler,
		QUADSPI_IRQHandler,
		LPTIM1_IRQHandler,
		CEC_IRQHandler,
		I2C4_EV_IRQHandler,
		I2C4_ER_IRQHandler,
		SPDIF_RX_IRQHandler,
		OTG_FS_EP1_OUT_IRQHandler,
		OTG_FS_EP1_IN_IRQHandler,
		OTG_FS_WKUP_IRQHandler,
		OTG_FS_IRQHandler,
		DMAMUX1_OVR_IRQHandler,
		HRTIM1_Master_IRQHandler,
		HRTIM1_TIMA_IRQHandler,
		HRTIM1_TIMB_IRQHandler,
		HRTIM1_TIMC_IRQHandler,
		HRTIM1_TIMD_IRQHandler,
		HRTIM1_TIME_IRQHandler,
		HRTIM1_FLT_IRQHandler,
		DFSDM1_FLT0_IRQHandler,
		DFSDM1_FLT1_IRQHandler,
		DFSDM1_FLT2_IRQHandler,
		DFSDM1_FLT3_IRQHandler,
		SAI3_IRQHandler,
		SWPMI1_IRQHandler,
		TIM15_IRQHandler,
		TIM16_IRQHandler,
		TIM17_IRQHandler,
		MDIOS_WKUP_IRQHandler,
		MDIOS_IRQHandler,
		JPEG_IRQHandler,
		MDMA_IRQHandler,
		0,
		SDMMC2_IRQHandler,
		HSEM1_IRQHandler,
		0,
		ADC3_IRQHandler,
		DMAMUX2_OVR_IRQHandler,
		BDMA_Channel0_IRQHandler,
		BDMA_Channel1_IRQHandler,
		BDMA_Channel2_IRQHandler,
		BDMA_Channel3_IRQHandler,
		BDMA_Channel4_IRQHandler,
		BDMA_Channel5_IRQHandler,
		BDMA_Channel6_IRQHandler,
		BDMA_Channel7_IRQHandler,
		COMP1_IRQHandler,
		LPTIM2_IRQHandler,
		LPTIM3_IRQHandler,
		LPTIM4_IRQHandler,
		LPTIM5_IRQHandler,
		LPUART1_IRQHandler,
		0,
		CRS_IRQHandler,
		0,
		SAI4_IRQHandler,
		0,
		0,
		WAKEUP_PIN_IRQHandler
};


/*================= Function definitions ==================*/
void Default_Handler() {

    // Go into an infinite loop.
    while(1) {

    }
}

void Reset_Handler() {

	/* set stack pointer */
    __asm("		ldr		sp, =_estack");

    /* Copy the data segment initializers from flash to SRAM */
    uint32_t* pui32Src, *pui32Dest;
    pui32Src = &_sidata;
    for(pui32Dest = &_sdata; pui32Dest < &_edata; )
        *pui32Dest++ = *pui32Src++;

    /* Zero fill the bss segment */
    __asm("    ldr     r0, =_sbss\n"
          "    ldr     r1, =_ebss\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");

    /* Call the clock system initialization function */
    SystemInit();
    /* Call static constructors */
    __libc_init_array();

    /* Call the application's entry point */
    main();
}

void NMI_Handler() {

    /* Go into an infinite loop */
    while(1) {

    }
}

void HardFault_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}

void MemManage_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}

void BusFault_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}

void UsageFault_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}

void SVC_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}

void DebugMon_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}

void PendSV_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}

void SysTick_Handler() {

	/* Go into an infinite loop */
    while(1) {

    }
}


static void SystemInit(void) {

	/* Enable the floating-point unit. Any configuration of the
	floating-point unit must be done here prior to it being enabled */
	HWREG(0xE000ED88) = ((HWREG(0xE000ED88) & ~0x00F00000) | 0x00F00000);

	/*------- Reset the RCC clock configuration to the default reset state -------*/
	/* Set HSION bit */
	RCC->CR |= 0x00000001;
	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;
	/* Reset HSEON, CSSON , CSION,RC48ON, CSIKERON PLL1ON, PLL2ON and PLL3ON bits */
	RCC->CR &= (uint32_t)0xEAF6ED7F;
	/* Reset D1CFGR register */
	RCC->D1CFGR = 0x00000000;
	/* Reset D2CFGR register */
	RCC->D2CFGR = 0x00000000;
	/* Reset D3CFGR register */
	RCC->D3CFGR = 0x00000000;
	/* Reset PLLCKSELR register */
	RCC->PLLCKSELR = 0x00000000;
	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x00000000;
	/* Reset PLL1DIVR register */
	RCC->PLL1DIVR = 0x00000000;
	/* Reset PLL1FRACR register */
	RCC->PLL1FRACR = 0x00000000;
	/* Reset PLL2DIVR register */
	RCC->PLL2DIVR = 0x00000000;
	/* Reset PLL2FRACR register */
	RCC->PLL2FRACR = 0x00000000;
	/* Reset PLL3DIVR register */
	RCC->PLL3DIVR = 0x00000000;
	/* Reset PLL3FRACR register */
	RCC->PLL3FRACR = 0x00000000;
	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;
	/* Disable all interrupts */
	RCC->CIER = 0x00000000;

	/* Change the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7) */
	HWREG(0x51008108) = 0x000000001;
}

