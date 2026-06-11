#define RCC_APB2ENR *(volatile unsigned int*)0x40021018

#define GPIOA_CRH *(volatile unsigned int*)0x40010804

#define USART1 *(volatile unsigned int*)0x40013800
#define USART1_BRR *(volatile unsigned int*)0x40013808
#define USART1_CR1 *(volatile unsigned int*)0x4001380C
#define USART1_DR *(volatile unsigned int*)0x40013804

static volatile unsigned int Baud_rate=8000000/(16*9600);

int main(){
	RCC_APB2ENR=RCC_APB2ENR&(~(1<<2));/*включаем тактирвоание на порте гпио А сначала обнуляем биты потом вставляем*/
	RCC_APB2ENR=RCC_APB2ENR|(1<<2);

	RCC_APB2ENR=RCC_APB2ENR & (~(1<<14));/*вклчаем тактирвоание ЮСАРТ 1 сначала обнуляем биты потом устанавливаем*/
	RCC_APB2ENR=RCC_APB2ENR | (1<<14);

	GPIOA_CRH=GPIOA_CRH & (~(0xB<<4));/*PA9 TX in ALTERNATE FUCNTION PUSH-UP*/
	GPIOA_CRH=GPIOA_CRH | (0xB<<4);

	GPIOA_CRH=GPIOA_CRH & (~(0x4<<8));/*Enabling PA10 in floating input*/
	GPIOA_CRH=GPIOA_CRH | (0x4<<8);


	/*Configuring USART1*/

	USART1_BRR = (52<<4)|1;

	USART1_CR1= USART1_CR1 & (~(1<<13)) & (~(1<<3)) & (~(1<<2));
	USART1_CR1= USART1_CR1 | (1<<13) | (1<<3) | (1<<2);

	while(1){
	
		while(!((USART1>>5)&1));/*Ждем пока Не RXNE не установитсья в 1  становитсья 1 когда в др приходит новый байт*/

		unsigned char received_byte=(unsigned char)(USART1_DR & 0xFF);/*Используем маску и для высечения байта и сохранения его в  переменной*/

		while(!((USART1>>7)&1));/*Становиться 1 когда можно записать следующий байт ждем пока он не станет 1*/

		USART1_DR=received_byte;/*Передаем Считаный байт обратно в юарт*/
		for(int i=0;i<100000;i++);
	}


	


}
