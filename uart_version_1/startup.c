#include <stdint.h>

/*External markers from Linker.ld*/
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

/*main prototype*/
int main(void);

/*Function that will works first when everything is turning on*/
void Reset_handler(void){
	/*Коприуем инциализирвоаныее перменные бсс из ФЛЕШ В РАМ пока не надо но на будущее надо*/
	uint32_t * bss_ptr=&_sbss;
	/*Обнуляем секцию bss*/
	while (bss_ptr<&_ebss){
		*bss_ptr++=0;
	}
	
	/*Jumping into main*/
	main();
	
	/*если мейн когда-то завершиться будем прыгать тут!!*/
	while(1);
}

/*Задаем адресс конца Стека(Верхушка RAM)*/

#define STACK_TOP 0x20005000//СТек на потолке каждые новые данные падают ему на дно

__attribute__((section(".isr_vector")))
void * vectors[]={
	(void*)STACK_TOP,
	Reset_handler
};



