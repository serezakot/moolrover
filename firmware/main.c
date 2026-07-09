/*
=======================================================================================================================================
                                      АНАТОМИЯ ПРОШИВКИ ЛУНОХОДА-САМОСВАЛА (ШПАРГАЛКА ДЛЯ СЕБЯ)
=======================================================================================================================================

1. СЛОЙ ПРИЕМА (Нервная система): ring_buffer, push_buf(), pop_buf()
   - Вход: Сырые поштучные байты из прерывания UART от Малинки.
   - Логика: Байты по одному складываются в массив. Указатели head и tail бегают по кругу через маску &(size-1).
   - Выход: Надежно сохраненный поток байт в оперативной памяти STM32.
   - Зачем: Чтобы контроллер не терял байты от Малинки, пока он занят расчетом ПИД-регулятора и кручением моторов.

2. СЛОЙ ПАРСИНГА (Фильтр мусора): parse_and_pack(), is_crc_valid()
   - Вход: Сырые байты, накопившиеся в кольцевом буфере.
   - Логика: Роутер ищет маркер старта пакета 0xAA. Если нашел, заглядывает на 6 байт вперед и ищет маркер конца 0xBB.
            Если маркеры на месте, берет первые 4 байта данных, шарашит их через XOR (^), проверяя контрольную сумму в 5-м байте.
   - Выход: Очищенная от мусора структура byte_packet с проверенными аргументами.
   - Зачем: Защищает робота от помех в проводах. Робот никогда не выполнит битую или случайную команду.

3. СЛОЙ ДИСПОТЧЕРА (Мозг робота): execute_table(), switch(command_id)
   - Вход: Проверенный byte_packet (ID команды + аргументы).
   - Логика: Распределяет задачи по железным узлам:
            * 0x00 (STOP)  -> Сброс скоростей и накопленных интегралов ПИДа в 0.0f, чтобы робот не дергался.
            * 0x01 (SPEED) -> Кастует байты в знаковый int8_t (для заднего хода) и превращает во float для моторов.
            * 0x03 (PID)   -> Собирает float коэффициента из целой и дробной части: args[2] + (args[3] / 100.0f).
            * 0x05 (SERVO) -> Проверяет угол (не больше 180) и пишет уставку для ковша (id 0) или кузова-самосвала (id 1).
   - Выход: Обновленные глобальные переменные целей (target_speed, target_servo_angles).
   - Зачем: Переводит цифровой язык Малинки в физические приказы для исполнительных механизмов.

4. СЛОЙ БЕЗОПАСНОСТИ (Предохранитель): check_conection_timeout()
   - Вход: Системное время (current_system_time) и время прихода последнего пакета (last_packet_received_time).
   - Логика: Если Малинка молчит и разница между таймерами переваливает за 300 миллисекунд.
   - Выход: Принудительный сброс целевых скоростей обоих бортов в 0.0f.
   - Зачем: Защита от крушения. Если оборвется провод связи, робот не улетит на полной скорости в стену, а послушно замрет.

5. СЛОЙ ТЕЛЕМЕТРИИ (Обратная связь): send_to_raspbery_telemetry(), rle_compression()
   - Вход: Структура raw_telemetry (текущие скорости, заряд батареи, датчики).
   - Логика: Сжимает идущие подряд одинаковые байты (например, нули полей резерва) алгоритмом RLE в пары [значение, сколько раз].
            Считает общую CRC, оборачивает в маркеры отправки 0xCC (старт) и 0xDD (конец) и выплевывает в UART.
   - Выход: Сжатый, легкий пакет данных, улетевший обратно на Малинку.
   - Зачем: Экономит время процессора. Меньше байт летит по проводу — быстрее STM32 возвращается к рулению.

---------------------------------------------------------------------------------------------------------------------------------------
КРАТКИЙ ИТОГ ДЛЯ СЕБЯ НА ЗАВТРА:
Логический конвейер полностью собран: Поймали байты -> Отсеяли мусор -> Проверили CRC -> Выполнили команду -> Проверили безопасность.
Завтра со свежей головой заходим в int main(void), настраиваем тактирование (RCC), миллисекунды (SysTick) и заводим этот луноход!
=======================================================================================================================================
*/




#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include <stdint.h>

#define MAX_RLE_PAIRS 16//максимальное количество пар скомперсированых рле алгоритмом
#define COMMAND_TIMEOUT_MS 300//время после которого будет считаться что распбери пай не дает команды - скорость в ноль

//COMMAND STATES
#define CMD_EMERGENCY_STOP 0x00
#define CMD_SET_SPEED 0x01
#define CMD_RESET_ODOMETER 0x02
#define CMD_TUNING_PID 0x03
#define CMD_SET_PERIPHERY 0x04
#define CMD_CONTROL_SERVOS 0x05

//atan2f 
float atan2f(float y,float x){
	float abs_y= y<0 ? -y:y;
	float res;
	if(x>y){
		res=(y-x)/(y+x);
		res =0.785398f - 0.785398f * res;
	}else{
		res=(y+x)/(x-y);
		res=2.35619f - 0.785398f * res;

	}
	return (y<0) ? -res:res;

}




//ENCODERS CONSTANTS

#define TICKS_PER_REV 120.0f
#define WHEEL_CIRCUMFERENCE 0.38683f//метров 

//global speed variables
float target_speed_left=0.0f;
float target_speed_right=0.0f;


uint32_t last_packet_received_time =0;
uint32_t current_system_time=0;

#define UART_BUFER_SIZE 4096
uint8_t raw_memory[UART_BUFER_SIZE];
//TIMEOUT ARCHITECTURE FOR SAFETY

void check_conection_timeout(uint32_t current_time,uint32_t last_time,float* target_speed_left,float* target_speed_right){
	if((current_time-last_time)>COMMAND_TIMEOUT_MS){
		*target_speed_left=0.0f;
		*target_speed_right=0.0f;
	}
}



//BUFFER RING ARCHITECTURE
typedef struct{
	uint8_t args[5];
}byte_packet;

typedef struct{
	uint8_t* buffer;
	uint16_t size;
	volatile uint16_t head;
	volatile uint16_t tail;	
}ring_buffer;

void init_buffer(ring_buffer* ring_buffer,uint16_t buffer_size){
	ring_buffer->head=ring_buffer->tail=0;

	ring_buffer->size=buffer_size;

}

uint8_t push_buf(ring_buffer* ring_buffer,uint8_t byte){
	
	uint16_t next_head=(ring_buffer->head+1)&(ring_buffer->size-1);
	if(next_head==ring_buffer->tail){
		return 0;
	}
	ring_buffer->buffer[ring_buffer->head]=byte;

	ring_buffer->head=next_head;
	return 1;
}

uint8_t pop_buf(ring_buffer* ring_buffer,uint8_t* data){
	if(ring_buffer->head==ring_buffer->tail)
		return 0;
	
	uint16_t temp_tail=ring_buffer->tail;

	ring_buffer->tail=(ring_buffer->tail+1)&(ring_buffer->size-1);
	*data=ring_buffer->buffer[temp_tail];
	return 1;
}

uint8_t is_empty(ring_buffer* ring_buffer){
	return ring_buffer->head == ring_buffer->tail;
}

uint16_t how_many_bytes(ring_buffer* ring_buffer){
	uint16_t result;
	result= (ring_buffer->head >= ring_buffer->tail) ? 
		(ring_buffer->head - ring_buffer->tail):
		(ring_buffer->size + ring_buffer->head - ring_buffer->tail);
	return result;
}

uint8_t is_more_than_7 (ring_buffer* ring_buffer){
	return how_many_bytes(ring_buffer) >= 7;
}

uint8_t is_it_data_packet(ring_buffer* ring_buffer){
	if((ring_buffer->buffer[ring_buffer->tail]) == 0xAA)
	{
		if(is_more_than_7(ring_buffer)){
			if((ring_buffer->buffer[(ring_buffer->tail+6)&(ring_buffer->size-1)]) == 0xBB)
				return 1;
			else
				return 0;
		}
		else
			return 0;
	}
	else
		return 0;
}
uint8_t parse_and_pack(ring_buffer* ring_buffer,byte_packet* command_pack){
	uint8_t packet_found_flag =0;
	while(how_many_bytes(ring_buffer)>=7){
		
		//if we found a data packet we are going out of cycle and packing our structure
		if(is_it_data_packet(ring_buffer)){
			packet_found_flag=1;
				break;
		}
		//moving our tail to next chek ups 
		ring_buffer->tail=(ring_buffer->tail+1)&(ring_buffer->size-1);
	}
	//we found a packet
	if(packet_found_flag){
		for(volatile int i=0;i<5;i++){
			uint16_t data_index=(ring_buffer->tail+1+i)&(ring_buffer->size-1);//записываем дату в буфер не считая начальный байт AA и конечный BB

			((uint8_t*)command_pack)[i]=ring_buffer->buffer[data_index];


		}
		//moving our tail to next 7 bytes right after 0xBB
		ring_buffer->tail=(ring_buffer->tail+7)&(ring_buffer->size-1);
		return 1;
	}
	else{
		return 0;
	}
}

uint8_t is_crc_valid(byte_packet* data_pack){
	uint8_t result;

	result=(data_pack->args[0] ^ data_pack->args[1] ^ data_pack->args[2] ^ data_pack->args[3])==data_pack->args[4];
	return result;
	//можно использовать и другие алгоритмы проверки сохраности срс суть в том чтобы значения после алгоритма на стм и распбери пай совпадали 
}

//TELEMETRY SENDING ->RASPBERY ARCHITECTURE
//
typedef struct {
	uint8_t status;
	uint8_t battery;
	int8_t left_speed;
	int8_t right_speed;
	uint8_t sonar_distance;
	uint8_t reserved[5];
}raw_telemetry;

typedef struct{
	uint8_t command;
	uint8_t counter;
}compressed_rle_pair;


void sent_to_usart1(uint8_t byte);
void sent_to_usart2(uint8_t byte);

void pack_data(compressed_rle_pair* dest,uint8_t comm,uint8_t count){
	dest->command=comm;
	dest->counter=count;
}

uint16_t rle_compression(const uint8_t* telemetry,uint16_t telemetry_size,compressed_rle_pair* out_buffer,uint16_t max_out_size){
	
	if(telemetry_size==0)
		return 0;
	uint16_t j=0;
	uint16_t current_command=telemetry[0];
	uint16_t counter=1;

	for(uint16_t i=1;i<telemetry_size;i++){
		if(j>=max_out_size){return j;}

		if(telemetry[i]!=telemetry[i-1]){
			pack_data(&out_buffer[j],current_command,counter);
			j+=1;

			current_command=telemetry[i];
			counter=1;
		}
		else {
			if(counter==255){
			pack_data(&out_buffer[j],current_command,counter);
			j++;
			counter=0;
			}
		counter+=1;	
		}
	}
	if(j<max_out_size){
		pack_data(&out_buffer[j],current_command,counter);
		j++;
	}
	return j;
}

void  send_to_raspbery_telemetry(raw_telemetry* raw_data){
	compressed_rle_pair rle_pairs[MAX_RLE_PAIRS];

	//сжимаем для отправки
	//
	uint16_t pairs_count=rle_compression((uint8_t*)raw_data,sizeof(raw_telemetry),rle_pairs,MAX_RLE_PAIRS);
	uint16_t compressed_bytes_len=pairs_count*sizeof(compressed_rle_pair);


	//считаем итоговый срс
	uint8_t telemetry_crc=(uint8_t)compressed_bytes_len;
	uint8_t* raw_bytes_ptr=(uint8_t*)rle_pairs;

	for(int i=0;i<compressed_bytes_len;i++){
		telemetry_crc^=raw_bytes_ptr[i];
	}
		
	//отправляем наш пакеееетик
	sent_to_usart1(0xCC);
	sent_to_usart1((uint8_t)compressed_bytes_len);
	for(int i=0;i<compressed_bytes_len;i++)
	{
		sent_to_usart1(raw_bytes_ptr[i]);
	}
	sent_to_usart1(telemetry_crc);
	sent_to_usart1(0xDD);
}

//MOTOR ARCHITECTURE
//
typedef struct{
	float current_speed;
	float odometer;

	float mass;
	float max_speed;
	float k_motor_q;
	float static_friction_q;
	float k_viscous_q;

}motor;

typedef struct{
	float Kp,Ki,Kd;

	float Integral;
	float previous_error;

	float max_output;
	float min_output;
}pid_controller;



//Motor configration and algho
void init_motor(motor* my_motor,float mass,float max_speed,float k_motor_q, float static_friction_q,float k_viscous_q)
{
	my_motor->mass=mass;
	my_motor->max_speed=max_speed;

	my_motor->k_motor_q=k_motor_q;
	my_motor->static_friction_q=static_friction_q;
	my_motor->k_viscous_q=k_viscous_q;


	my_motor->odometer=0.0f;
	my_motor->current_speed=0.0f;
	
}
/* Симуляция работы мотора можно использовать как килер фичу для потна
void motor_update(motor* my_motore,float pwm,float dt){
	float motor_force,friction_force, acceleration;

	//сила тяги попрорциональная шиму
	motor_force=pwm*(my_motor->k_motor_q);
	//сила сопротивления
	friction_force=(my_motor->static_friction_q) + (my_motor->current_speed * my_motor->k_viscous_q);
	//ускорение по ньютону
	acceleration=(motor_force-friction_force)/(my_motor->mass);

	my_motor->current_speed=accleration*dt;
	if((my_motor->current_speed) > (my_motor->max_speed)) my_motor->current_speed=my_motor->max_speed;

	my_motor->odometer=+my_motor->current_speed*dt;


}
*/
//Pid configuration and algho
void pid_init(pid_controller* pid,float Kp,float Ki,float Kd,float max_output,float min_output){
	pid->Kp=Kp;
	pid->Ki=Ki;
	pid->Kd=Kd;

	pid->max_output=max_output;
	pid->min_output=min_output;

	pid->Integral=0.0f;
	pid->previous_error=0.0f;
}

float pid_compute(pid_controller* pid,float set_point,float feedback, float dt){
	if(dt<=0.0f) return 0.0f;//защита от деления на ноль / отрицательного dt

	float error=set_point-feedback;
	
	float derivative=(error-pid->previous_error)/dt;

	pid->Integral=pid->Integral + error*dt;
	if((pid->Integral) > (pid->max_output)) pid->Integral = pid->max_output; //positive saturating
	else if ((pid->Integral) < (pid->min_output)) pid->Integral = pid->min_output; //negative saturating
	
	float output=(error*(pid->Kp)) + (pid->Integral*(pid->Ki)) + (derivative*(pid->Kd));

	if(output > (pid->max_output)) output = pid->max_output;
	else if (output < (pid->min_output)) output = pid->min_output;
	
	pid->previous_error=error;

	return output;
}



//COMMAND EXECUTION ARCHITECTURE

void  set_servo_angles(uint8_t servo_id,float servo_angle);


float target_servo_agnlees[4]={0.0f,0.0f,0.0f,0.0f};

pid_controller pid_left,pid_right;
motor motor_left,motor_right;


void execute_table(byte_packet* packet){
	uint8_t command_id=packet->args[0];
	//reseting TIMEOUT checking	
	last_packet_received_time=current_system_time;
	
	float new_val;
	uint8_t target_coef;

	uint8_t servo_id;
	float servo_angle;

	switch(command_id){
		case CMD_EMERGENCY_STOP:
			target_speed_left=0.0f;
			target_speed_right=0.0f;
			pid_left.Integral=0.0f;
			pid_right.Integral=0.0f;	
		break;

		case CMD_SET_SPEED:
			//кастуем к инту потом к флоату кастуем к инту для возможной отирцательной скорости
			target_speed_left=((float)(int8_t)packet->args[1]);
			target_speed_right=((float)(int8_t)packet->args[2]);
		break;
		
		case CMD_RESET_ODOMETER:
			motor_left.odometer=0.0f;
			motor_right.odometer=0.0f;
		break;

		case CMD_TUNING_PID:

			new_val=packet->args[2]+(packet->args[3]/100.0f);
			target_coef=packet->args[1];

			if(target_coef==1){
				pid_left.Kp=new_val;
				pid_right.Kp=new_val;
			}
			else if(target_coef==2){
				pid_left.Ki=new_val;
				pid_left.Integral=0.0f;

				pid_right.Ki=new_val;
				pid_right.Integral=0.0f;
			}
			else if(target_coef==3){
				pid_left.Kd=new_val;
				pid_right.Kd=new_val;
			}
		break;

		case CMD_CONTROL_SERVOS:
			servo_id=packet->args[1];
			servo_angle=packet->args[2];	
			if(servo_angle>180.0f)
				servo_angle=180.0f;
			if(servo_id<4){
				target_servo_agnlees[servo_id]=servo_angle;
				set_servo_angles(servo_id,servo_angle);
				}
		break;	
		
		default:

		break;
	}
}



//STM32 ARCHITECTURE

//SYS_TICK INITALIZATION
void SysTick_initialization(void){

	SysTick->LOAD=71999U;//Value of reseting
	SysTick->VAL=0U;

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | //тактирование от ядра 72Mhz
			 SysTick_CTRL_TICKINT_Msk   |//генерировать прерывания при дохода до нуля
			 SysTick_CTRL_ENABLE_Msk;    //Запуск счетчика

}

void SysTick_Handler(void){
	current_system_time++;
}

//USART1 CONFIGURATION AND ARCHITECTURE

ring_buffer uart_ring;

void init_uart1(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

	GPIOA->CRH &= ~ ((0xF<<4) | (0XF<<8));
	GPIOA->CRH |= (0xB<<4);//pa9 push-pull mode 1011
	GPIOA->CRH |= (0x4<<8);//Floating mod 0100

	USART1->BRR=0x271;//115200 bod

	USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

	//разрешаем прерывания	
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void){
	if(USART1->SR & USART_SR_RXNE){
		uint8_t incoming_byte=(uint8_t)(USART1->DR &0xFF);

		push_buf(&uart_ring, incoming_byte);
	}
}
//USART TX (блокирующая отправка одного байта, нужна телеметрии)
void sent_to_usart1(uint8_t byte){
	while(!(USART1->SR & USART_SR_TXE));
	USART1->DR=byte;
}
void sent_to_usart2(uint8_t byte){
	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR=byte;
}

//MOTORS STM ARCHITECTURE

void init_motors(void){
	
	RCC->APB2ENR |=RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	//PA 8 PA 7 ALTERNATE FUCNTION	
	GPIOA->CRL &= ~((0xf<<24)|(0xf<<28));
	GPIOA->CRL |=((0xb<<24)|(0xb<<28));
	
	//PB 0 PB 1 обычный выход
	GPIOB->CRL &= ~((0xf<<0)|(0xf<<4));
	GPIOB->CRL |= ((0x3<<0) | (0x3<<4));
	
	TIM3->PSC = 0;
	TIM3->ARR = 2999;
	
	//пвм мод 1 для 1 и 2 канала
	TIM3->CCMR1 |= (0x6<<TIM_CCMR1_OC1M_Pos) | (0x6<<TIM_CCMR1_OC2M_Pos) ;
	//разрешение на вывод сигнала 	
	TIM3->CCER |=TIM_CCER_CC1E | TIM_CCER_CC2E;	

	TIM3->CR1|=TIM_CR1_CEN;	

	TIM3->CCR1=2999;
	TIM3->CCR2=2999;
}

void drive_motors(float left_pwm,float right_pwm){
	//left part
	
	if(left_pwm>=0.0f){

		GPIOB->BSRR=GPIO_BSRR_BS0;//PB0 в 1(вперед)
	}
	else{
			GPIOB->BSRR=GPIO_BSRR_BR0;//ставим 0 (назад)
			left_pwm=-left_pwm;
	}

	if(left_pwm>100.0f) left_pwm=100.0f;

	//расчет шим
	TIM3->CCR1 =2999-(uint32_t)(left_pwm*29.99f);

	//right part
	
	if(right_pwm>=0.0f){
		GPIOB->BSRR=GPIO_BSRR_BS1;
	}
	else{
		GPIOB->BSRR=GPIO_BSRR_BR1;
		right_pwm=-right_pwm;
	}
	if(right_pwm>100.0f) right_pwm=100.0f;
	
	TIM3->CCR2=2999-(uint32_t)(right_pwm*29.99f);	
}


//ENCODERS ARCHITECTURE

volatile int32_t encoder_tick_left = 0;
volatile int32_t encoder_tick_right = 0 ;

void init_encoders(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

	//PC13 PC 14 configuration  input pull-up
	GPIOC->CRH &= ~((0xf<<20)|(0xf<<24));
	GPIOC->CRH |= ((0x8<<20)|(0x8<<24));
	
	//выбор пулл ап
	GPIOC->ODR |= (GPIO_ODR_ODR13) | (GPIO_ODR_ODR14);

	//связываем EXTI13 EXTI14 с портом GPIOC
	//EXITCR[3] отвечает за линии 12-15  0х2 означает порт С
	AFIO->EXTICR[3] &= ~((0xf<<4)|(0xf<<8));//очистка
	AFIO->EXTICR[3] |= ((0x2<<4)|(0x2<<8));// запись

	//ловим подьем сигнала РТСР и спат ФТСР
	EXTI->FTSR |= (EXTI_FTSR_TR13) | (EXTI_FTSR_TR14);
	EXTI->RTSR |= (EXTI_RTSR_TR13) | (EXTI_RTSR_TR14);

	//открытие маски прерыванияa
	EXTI->IMR |= (EXTI_IMR_MR13) | (EXTI_IMR_MR14);

	//разрешарм прерывание
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
//обработчик прерывания енкодеров
void EXTI15_10_IRQHandler(void){
	//прерывание случилось изза левого мотора ?(13)
	if(EXTI->PR & EXTI_PR_PR13){
		EXTI->PR =EXTI_PR_PR13;//записываем единиу для сброса флага прерывания
	
	if(GPIOB->ODR & GPIO_ODR_ODR0){
		encoder_tick_left++;//едем вперед
		}
	else {
		encoder_tick_left--;//едем назад
		}
	}
	//быть может прерывание случилось изза прваого моторa? (14)
	if(EXTI->PR &EXTI_PR_PR14){
		EXTI->PR = EXTI_PR_PR14;
			
		if(GPIOB->ODR &GPIO_ODR_ODR1){
			encoder_tick_right++;//вперед
		}
		else{
			encoder_tick_right--;//назад
		}

	}
}

void update_motorss(float dt){
	
	__disable_irq();
	int32_t ticks_l =encoder_tick_left;
	int32_t ticks_r = encoder_tick_right;
	encoder_tick_left = 0;
	encoder_tick_right = 0;
	__enable_irq();


	//перевод тиков в реальную физическую скорость. (тики/ всего тиков на оборот) * длина окружности колеса/ время
	motor_left.current_speed=(((float)ticks_l)/TICKS_PER_REV)*WHEEL_CIRCUMFERENCE/dt;
	motor_right.current_speed=(((float)ticks_r)/TICKS_PER_REV)*WHEEL_CIRCUMFERENCE/dt;

	//считаем одометрию
	motor_left.odometer+=motor_left.current_speed*dt;
	motor_right.odometer+=motor_right.current_speed*dt;

	//переводим реал скорость в пид регуляторы)
	
	float pid_out_left=pid_compute(&pid_left,target_speed_left,motor_left.current_speed, dt);
	float pid_out_rright=pid_compute(&pid_right,target_speed_right,motor_right.current_speed, dt);

	//отправялем шим
	drive_motors(pid_out_left,pid_out_rright);
}
//SERVOS ARCHITECTURE

void init_servos(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	//configruing PA0 PA1 PA2 PA3 alt funciton out pp (0xb)
	
	GPIOA->CRL &= ~(0xffff);
	GPIOA->CRL |= (0xb<<0)|(0xb<<4)|(0xb<<8)|(0xb<<12);	

	TIM2->PSC=71;//1 мкс
	TIM2->ARR=19999;// 20мс(50мгц)
	
	TIM2->CCMR1 |= (0x6<<TIM_CCMR1_OC1M_Pos) | (0x6<<TIM_CCMR1_OC2M_Pos);
	TIM2->CCMR2 |= (0x6<<TIM_CCMR2_OC3M_Pos) | (0x6<<TIM_CCMR2_OC4M_Pos);

	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM2->CR1 |= TIM_CR1_CEN;

	TIM2->CCR1 =1500;
	TIM2->CCR2 =1500;
	TIM2->CCR3 =1500;
	TIM2->CCR4 =1500;
}

void set_servo_angles(uint8_t servo_id,float angle){
	uint32_t width = 1000+ (uint32_t)(angle*(1000.0f/180.0f));

	switch(servo_id){
	case 0: TIM2->CCR1 = width; break;//PA0
	case 1: TIM2->CCR2 = width; break;//PA1
	case 2: TIM2->CCR3 =width;  break;//PA2
	case 3: TIM2->CCR4 = width; break;//PA3
	default: break;
	}
}


//IMU ARCHITECTURE
//
typedef struct{
	//переменыне для хранения данных с датчика
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t mx, my, mz;
	
	//фильтрованые углы ориентации
	float roll;
	float pitch;
	float yaw;// угол курса

}imu_data;

typedef struct{
	float roll,pitch,yaw;
	float gyro_bias_z;
}robot_orientation;

robot_orientation robot_pose={0};

imu_data imu;

void init_imu_i2c(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	//PB6(scl) PB7(sda) alternate fucntion open_drain (0xe)
	
	GPIOB->CRL &=~((0xf<<24)|(0xf<<28));
	GPIOB->CRL |=(0xe<<24)|(0xe<<28);

	I2C1->CR1 |=I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	I2C1->CR2 |= 36;//частота АПБ1 переферии =36 мгц
	I2C1->CCR = I2C_CCR_FS | 30;//коефициент для 400кГц
	I2C1->TRISE = 12;//максимальное время подьема сигнала

	//влключение модуля айтуси
	I2C1->CR1 |=I2C_CR1_PE;
}

#define I2C_TIMEOUT_ITERS 100000U//грубый счётчик итераций до выхода по таймауту

//ждём флаг в SR1 с таймаутом. 1 = дождались, 0 = вышли по таймауту (потеря связи с IMU)
uint8_t i2c_wait_sr1(uint32_t flag){
	uint32_t t=I2C_TIMEOUT_ITERS;
	while(!(I2C1->SR1 & flag)){
		if(--t==0) return 0;
	}
	return 1;
}

uint8_t i2c_init(uint8_t dev_addr,uint8_t reg_addr,uint8_t value){
	//старт
	I2C1->CR1 |= I2C_CR1_START;
	if(!i2c_wait_sr1(I2C_SR1_SB)) return 0;

	I2C1->DR = dev_addr<<1;//I2C имеет строгий 1 байтный протоко дев аддр у нас 7 битный адресс младший бит отвечаает за запись и чтение при сдвиге у нас получаеться уже 8 битный адресс с младшим битом равным нулю который будет использоваться для записи в него 1 или 0

	if(!i2c_wait_sr1(I2C_SR1_ADDR)){ I2C1->CR1|=I2C_CR1_STOP; return 0; }
	(void)I2C1->SR2;

	//отправка адресса регистар
	I2C1->DR=reg_addr;
	if(!i2c_wait_sr1(I2C_SR1_TXE)){ I2C1->CR1|=I2C_CR1_STOP; return 0; }

	I2C1->DR=value;//РАНЬШЕ value вообще не отправлялся — регистр писался, значение нет
	if(!i2c_wait_sr1(I2C_SR1_BTF)){ I2C1->CR1|=I2C_CR1_STOP; return 0; }

	//stop
	I2C1->CR1 |= I2C_CR1_STOP;
	return 1;
}

//burst-чтение n байт начиная с reg_addr; репит-старт, NACK+STOP на последнем байте
uint8_t i2c_burst_read(uint8_t dev_addr,uint8_t reg_addr,uint8_t* buf,uint8_t n){
	if(n==0) return 0;

	//фаза записи адреса регистра
	I2C1->CR1 |= I2C_CR1_START;
	if(!i2c_wait_sr1(I2C_SR1_SB)) return 0;
	I2C1->DR = dev_addr<<1;
	if(!i2c_wait_sr1(I2C_SR1_ADDR)){ I2C1->CR1|=I2C_CR1_STOP; return 0; }
	(void)I2C1->SR2;
	I2C1->DR = reg_addr;
	if(!i2c_wait_sr1(I2C_SR1_TXE)){ I2C1->CR1|=I2C_CR1_STOP; return 0; }

	//репит-старт на чтение
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	if(!i2c_wait_sr1(I2C_SR1_SB)) return 0;
	I2C1->DR = (dev_addr<<1)|1;
	if(!i2c_wait_sr1(I2C_SR1_ADDR)){ I2C1->CR1|=I2C_CR1_STOP; return 0; }
	(void)I2C1->SR2;

	for(uint8_t i=0;i<n;i++){
		if(i==(n-1)){
			I2C1->CR1 &= ~I2C_CR1_ACK;//NACK перед последним байтом
			I2C1->CR1 |= I2C_CR1_STOP;
		}
		if(!i2c_wait_sr1(I2C_SR1_RXNE)){ I2C1->CR1|=I2C_CR1_STOP; return 0; }
		buf[i]=(uint8_t)I2C1->DR;
	}
	return 1;
}

void init_imu_sensors(void){
	init_imu_i2c();
	
	//настройки МПУ6050
	i2c_init(0x68,0x6b,0x00);//запускаем чип
	i2c_init(0x68,0x1b,0x08);//гироскоп  режим +-500 градусов в сек
	i2c_init(0x68,0x1c,0x08);//акселерометр в режиме 4г

	//настройки HMCL883l
	
	i2c_init(0x1e,0x00,0x70);//8-average 15гц 
	i2c_init(0x1e,0x01,0x20);//gain=1.3ga
	i2c_init(0x1e,0x02,0x00);//Непрерывный режим измерения	
}






//IMU READING ARCHITECTURE

#define GYRO_LSB_PER_DPS 65.5f//чувствительность гироскопа в режиме +-500 град/с
#define COMP_ALPHA 0.98f//вес гироскопа в комплементарном фильтре

//burst-чтение сырых данных MPU6050 (accel+gyro) и HMC5883 (mag)
void imu_read_raw(void){
	uint8_t b[14];
	//0x3B: AX,AY,AZ,TEMP,GX,GY,GZ по 2 байта, старший байт первым
	if(i2c_burst_read(0x68,0x3B,b,14)){
		imu.ax=(int16_t)((b[0]<<8)|b[1]);
		imu.ay=(int16_t)((b[2]<<8)|b[3]);
		imu.az=(int16_t)((b[4]<<8)|b[5]);
		imu.gx=(int16_t)((b[8]<<8)|b[9]);
		imu.gy=(int16_t)((b[10]<<8)|b[11]);
		imu.gz=(int16_t)((b[12]<<8)|b[13]);
	}
	uint8_t m[6];
	//HMC5883 отдаёт оси в порядке X,Z,Y
	if(i2c_burst_read(0x1e,0x03,m,6)){
		imu.mx=(int16_t)((m[0]<<8)|m[1]);
		imu.mz=(int16_t)((m[2]<<8)|m[3]);
		imu.my=(int16_t)((m[4]<<8)|m[5]);
	}
}

//комплементарный фильтр курса: интеграл гироскопа + подмешивание магнитометра
void imu_update(float dt){
	imu_read_raw();
	float gz_dps=((float)imu.gz)/GYRO_LSB_PER_DPS - robot_pose.gyro_bias_z;
	float yaw_mag=atan2f((float)imu.my,(float)imu.mx)*(180.0f/3.14159265f);
	robot_pose.yaw=COMP_ALPHA*(robot_pose.yaw+gz_dps*dt)+(1.0f-COMP_ALPHA)*yaw_mag;
	imu.yaw=robot_pose.yaw;
}

int main(void){
	SysTick_initialization();

	init_buffer(&uart_ring,UART_BUFER_SIZE);
	uart_ring.buffer=raw_memory;

	init_uart1();
	init_motors();
	init_servos();
	init_encoders();//ВРЕМЕННО на EXTI (см. заметку по задаче 1 — до перехода на таймеры)
	init_imu_sensors();//IMU по I2C (теперь с таймаутами и реальной отправкой value)


	//math and pid`s init
	//
	pid_init(&pid_left,1.0f,0.2f,0.05f,100.0f,-100.0f);
	pid_init(&pid_right,1.0f,0.2f,0.05f,100.0f,-100.0f);


	init_motor(&motor_left,2.5f,100.0f,1.2f,5.0f,0.8f);
	init_motor(&motor_right,2.5f,100.0f,1.2f,5.0f,0.8f);


	//метки диспетчера
	
	uint32_t last_locomotion_time =0;
	uint32_t last_telemetry_time =0;

	byte_packet current_packet;
	raw_telemetry telemetry_packet={0};//send to pi structure
	

	while(1){
		uint32_t now=current_system_time;
		// I
		if(parse_and_pack(&uart_ring,&current_packet))
		{ 
			if(is_crc_valid(&current_packet))
			{
				execute_table(&current_packet);
			}	   
				
		}
		// II
		check_conection_timeout(now,last_packet_received_time,&target_speed_left,&target_speed_right);
		
		//III
		
		if((now-last_locomotion_time)>=20){
			float dt=(now-last_locomotion_time)/1000.0f;
			last_locomotion_time =now;




		update_motorss(dt);
		imu_update(dt);//обновляем курс по IMU в том же такте 20 мс
		}	

		// IV
		if((now-last_telemetry_time)>=100){
			last_telemetry_time=now;

			telemetry_packet.status=0x01;
			telemetry_packet.battery=114;
			telemetry_packet.left_speed=(int8_t)motor_left.current_speed;
			telemetry_packet.right_speed=(int8_t)motor_right.current_speed;
			telemetry_packet.sonar_distance=50;

			send_to_raspbery_telemetry(&telemetry_packet);
		}	


}
}
