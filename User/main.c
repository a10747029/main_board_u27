#include "gd32f1x0.h"
#include <stdio.h>
#include "systick.h"
#define TIME_PLUS_MS 1000
void delay(int time)
{
	  int target_time = TIME_PLUS_MS * time;
    while(target_time--);
    
    return;
}
void delay_us(int time)
{
	  int target_time = time;
    while(target_time--);
    
    return;
}
void power_init(void);
void com_init0(void);
void com_init1(void);
void process_command(void);
void periodic_report(void);
uint8_t need_process_command = 0;
uint8_t need_periodic_report = 0;
uint8_t need_report_adc = 0;
uint8_t host_bootup_ok = 0;
extern uint8_t adc_value[20];
int main(void)
{		
		nvic_irq_enable(USART0_IRQn, 0, 0);
		nvic_irq_enable(USART1_IRQn, 0, 0);
    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV2);
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
		gpio_bit_reset(GPIOB, GPIO_PIN_2);//set:开启，reset：关闭蜂鸣器
	
	  rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
		gpio_bit_reset(GPIOC, GPIO_PIN_13);//set:开启，reset：关闭蜂鸣器
	
		com_init0();
		com_init1();
		power_init();
		BQ40Z50_Init();
    while(1)
    { 
			if(need_process_command)
			{
				process_command();
				need_process_command = 0;
			}
			if(need_periodic_report && host_bootup_ok)
			{
				periodic_report();
				need_periodic_report = 0;
			}
			#if 0
			if(need_report_adc)
			{
				printf("%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x.\r\n",adc_value[0],adc_value[1],adc_value[2],adc_value[3],adc_value[4],
																																									adc_value[5],adc_value[6],adc_value[7],adc_value[8],adc_value[9],
																																									adc_value[10],adc_value[11],adc_value[12],adc_value[13],adc_value[14],
																																									adc_value[15],adc_value[16],adc_value[17],adc_value[18],adc_value[19]);
				need_report_adc = 0;
			}
			#endif
			if(0){
				gpio_bit_reset(GPIOB, GPIO_PIN_2);
				gpio_bit_reset(GPIOC, GPIO_PIN_13);
				delay(100);
				gpio_bit_set(GPIOB, GPIO_PIN_2);
				gpio_bit_set(GPIOC, GPIO_PIN_13);
				delay_us(30);
			}
		}
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
