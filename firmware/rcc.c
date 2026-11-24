
#include "stm32f103xb.h"
#include "stm32f1xx.h"

void SystemClock_Config(void){
	RCC->CR |= RCC_CR_HSEON;
	
	while(!(RCC->CR & RCC_CR_HSERDY));


	//72 МГц требуют 2 такта ожидания флеша: LATENCY = 0b010.
	//ВНИМАНИЕ: FLASH_ACR_LATENCY_1 — это БИТ 1 (=0x2 = два такта),
	//а FLASH_ACR_LATENCY_2 — бит 2 (=0x4 = зарезервированное значение 100!).
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;


	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
	RCC->CFGR |= RCC_CFGR_PLLSRC;
	RCC->CFGR |= RCC_CFGR_PLLMULL9;

	RCC->CR |=RCC_CR_PLLON;
	
	while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

		


}
