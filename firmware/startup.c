#include <stdint.h>

/* Символы из linker_script.ld */
extern uint32_t _sidata;   /* значения .data во FLASH (адрес-источник, LMA) */
extern uint32_t _sdata;    /* начало .data в RAM */
extern uint32_t _edata;    /* конец .data в RAM */
extern uint32_t _sbss;     /* начало .bss в RAM */
extern uint32_t _ebss;     /* конец .bss в RAM */

int main(void);
void Reset_Handler(void);
void Default_Handler(void);

#define STACK_TOP 0x20005000U   /* вершина 20К RAM: 0x20000000 + 20K */

/* Слабые заглушки: реальные обработчики из main.c их перекроют */
void NMI_Handler(void)                __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)                __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)             __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)            __attribute__((weak, alias("Default_Handler")));
void WWDG_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void TAMPER_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void USB_HP_CAN1_TX_IRQHandler(void)  __attribute__((weak, alias("Default_Handler")));
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void USBWakeUp_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));

typedef void (*vector_entry)(void);

__attribute__((section(".isr_vector")))
const vector_entry vectors[] = {
	(vector_entry)STACK_TOP,   /* 0  начальный SP        */
	Reset_Handler,             /* 1  сброс               */
	NMI_Handler,               /* 2  */
	HardFault_Handler,         /* 3  */
	MemManage_Handler,         /* 4  */
	BusFault_Handler,          /* 5  */
	UsageFault_Handler,        /* 6  */
	0, 0, 0, 0,                /* 7-10 зарезервировано   */
	SVC_Handler,               /* 11 */
	DebugMon_Handler,          /* 12 */
	0,                         /* 13 зарезервировано     */
	PendSV_Handler,            /* 14 */
	SysTick_Handler,           /* 15 <- твой SysTick     */
	WWDG_IRQHandler,           /* 16 IRQ0                */
	PVD_IRQHandler,
	TAMPER_IRQHandler,
	RTC_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	DMA1_Channel5_IRQHandler,
	DMA1_Channel6_IRQHandler,
	DMA1_Channel7_IRQHandler,
	ADC1_2_IRQHandler,
	USB_HP_CAN1_TX_IRQHandler,
	USB_LP_CAN1_RX0_IRQHandler,
	CAN1_RX1_IRQHandler,
	CAN1_SCE_IRQHandler,
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
	USART1_IRQHandler,         /* 53 IRQ37 <- твой USART1 */
	USART2_IRQHandler,
	USART3_IRQHandler,
	EXTI15_10_IRQHandler,      /* 56 IRQ40 <- твои энкодеры */
	RTC_Alarm_IRQHandler,
	USBWakeUp_IRQHandler
};

void Reset_Handler(void){
	/* 1. Копируем .data из FLASH (LMA) в RAM (VMA) */
	uint32_t *src = &_sidata;
	uint32_t *dst = &_sdata;
	while (dst < &_edata) *dst++ = *src++;

	/* 2. Обнуляем .bss */
	uint32_t *bss = &_sbss;
	while (bss < &_ebss) *bss++ = 0;

	/* 3. СЛЕДУЮЩИЙ ШАГ: здесь, до main(), должна встать настройка тактовой
	      на 72 МГц (HSE+PLL). Пока её нет — чип на 8 МГц HSI. */

	main();

	while(1);   /* если main вернётся */
}

void Default_Handler(void){
	while(1);   /* необъявленное прерывание застрянет тут */
}