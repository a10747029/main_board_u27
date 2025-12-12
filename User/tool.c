#include "gd32f1x0_usart.h"
#include "string.h"
void com_init0()    // for PA9，PA10
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_9);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0,115200U);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
		usart_interrupt_enable(USART0, USART_INT_RBNE);
    usart_enable(USART0);
}

void com_init1()    // for PA15
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);

    /* connect port to USARTx_Tx */
//    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_15);

    /* configure USART Tx as alternate function push-pull */
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_2);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_2);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_15);

    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1,115200U);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
		usart_interrupt_enable(USART1, USART_INT_RBNE);
    usart_enable(USART1);
}
#define ADC_IN0_VCC24_MIN				   3217
#define ADC_IN0_VCC24_MAX          3574
#define ADC_IN1_VCC3V3_ULP_MIN     2247 
#define ADC_IN1_VCC3V3_ULP_MAX     2491
#define ADC_IN4_VBAT_MIN           2518
#define ADC_IN4_VBAT_MAX           3438
#define ADC_IN5_VCC12_SYS_MIN      3084
#define ADC_IN5_VCC12_SYS_MAX      3484
#define ADC_IN6_VCC5V0_CORE_MIN    3231
#define ADC_IN6_VCC5V0_CORE_MAX    3588
#define ADC_IN7_VCC5V0_SYS_MIN     3231
#define ADC_IN7_VCC5V0_SYS_MAX     3588
void delay(int time);
extern uint8_t adc_value[20];
void power_init()
{
		rcu_periph_clock_enable(RCU_GPIOA);
		  //3v3 ulp
	  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_0);
		gpio_bit_reset(GPIOA, GPIO_PIN_0);
			//5v0 sys
	  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_1);
		gpio_bit_reset(GPIOA, GPIO_PIN_1);
			//5v0 core
	  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_4);
		gpio_bit_reset(GPIOA, GPIO_PIN_4);
			//3v3 sys
	  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_5);
		gpio_bit_reset(GPIOA, GPIO_PIN_5);
			//12v sys
	  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_6);
		gpio_bit_reset(GPIOA, GPIO_PIN_6);
		  // another power
		gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,GPIO_PIN_7);
		gpio_bit_reset(GPIOA, GPIO_PIN_7);
		//hareware define boot sequence
		gpio_bit_set(GPIOA, GPIO_PIN_0);
		delay(10);
		gpio_bit_set(GPIOA, GPIO_PIN_7);
		delay(10);
		gpio_bit_set(GPIOA, GPIO_PIN_1);
		delay(25);
		gpio_bit_set(GPIOA, GPIO_PIN_6);
		delay(10);
		gpio_bit_set(GPIOA, GPIO_PIN_5);
		delay(100);
		#if 0
				if(((((adc_value[0] << 8) | adc_value[1]) < ADC_IN0_VCC24_MIN)         || (((adc_value[0] << 8) | adc_value[1]) > ADC_IN0_VCC24_MAX))       ||
					 ((((adc_value[2] << 8) | adc_value[3]) < ADC_IN1_VCC3V3_ULP_MIN)    || (((adc_value[2] << 8) | adc_value[3]) > ADC_IN1_VCC3V3_ULP_MAX))  ||
				   ((((adc_value[4] << 8) | adc_value[5]) < ADC_IN4_VBAT_MIN)          || (((adc_value[4] << 8) | adc_value[5]) > ADC_IN4_VBAT_MAX))        ||
				   ((((adc_value[6] << 8) | adc_value[7]) < ADC_IN5_VCC12_SYS_MIN)     || (((adc_value[6] << 8) | adc_value[7]) > ADC_IN5_VCC12_SYS_MAX))   ||
				   ((((adc_value[8] << 8) | adc_value[9]) < ADC_IN6_VCC5V0_CORE_MIN)   || (((adc_value[8] << 8) | adc_value[9]) > ADC_IN6_VCC5V0_CORE_MAX)) ||
				   ((((adc_value[10] << 8) | adc_value[11]) < ADC_IN7_VCC5V0_SYS_MIN)  || (((adc_value[10] << 8) | adc_value[11]) > ADC_IN7_VCC5V0_SYS_MAX)))
				{ 
					gpio_bit_reset(GPIOA, GPIO_PIN_4);
				}
				else{
					gpio_bit_set(GPIOA, GPIO_PIN_4);
				}
		#endif
		gpio_bit_set(GPIOA, GPIO_PIN_4);		
}
void usart_transmit(uint32_t usart_periph,uint8_t *data,uint8_t length)
{
	int i = 0;
	for(i = 0;i < length; i++)
	{  usart_data_transmit(usart_periph, *(data+i));
		 while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
	}
}

#define HOST_RECEIVE_COUNT 6
#define HOST_COMMAND_HEART_BEAT       0x00
#define HOST_COMMAND_GET_POWER_STATUS 0x01
#define HOST_COMMAND_SET_POWER_STATUS 0x02
#define HOST_COMMAND_GET_ADC_VALUE    0x03
#define HOST_COMMAND_GET_BAT_CAPACITY 0x04
#define HOST_COMMAND_GET_BAT_TEMPEARATURE 0x05
#define HOST_COMMAND_GET_BAT_VOLTAGE  0x06
#define HOST_COMMAND_BOOTUP_SUCCESS   0x10
extern uint8_t adc_value[];
extern uint8_t host_receiver_buffer[6];
extern uint8_t host_bootup_ok;
int battery_temp_data = -1;
uint8_t host_reply_heart_beat_buffer[6] =       {0x55,0xaa,HOST_COMMAND_HEART_BEAT,0x00,0x00,0xff};
uint8_t host_reply_get_power_status_buffer[6] = {0x55,0xaa,HOST_COMMAND_GET_POWER_STATUS,0x00,0x00,0xff};
uint8_t host_reply_set_power_status_buffer[6] = {0x55,0xaa,HOST_COMMAND_SET_POWER_STATUS,0x00,0x00,0xff};
uint8_t host_reply_get_adc_value_buffer[24] =   {0x55,0xaa,HOST_COMMAND_GET_ADC_VALUE,0x00,0x00,0x00,0x00,
                                                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
																								0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff};
uint8_t host_reply_get_bat_capacity_buffer[6] =       {0x55,0xaa,HOST_COMMAND_GET_BAT_CAPACITY,0x00,0x00,0xff};
uint8_t host_reply_get_bat_temperature_buffer[6] =       {0x55,0xaa,HOST_COMMAND_GET_BAT_TEMPEARATURE,0x00,0x00,0xff};
uint8_t host_reply_get_bat_voltage_buffer[6] =       {0x55,0xaa,HOST_COMMAND_GET_BAT_VOLTAGE,0x00,0x00,0xff};
uint8_t host_reply_bootup_success_buffer[6] =       {0x55,0xaa,HOST_COMMAND_BOOTUP_SUCCESS,0x00,0x00,0xff};
uint8_t host_reply_error_status_buffer[6] = {0x55,0xaa,0xff,0xff,0xff,0xff};
												   //3v3 ulp      5v0 sys   5v0 core      12v      3v3 sys
uint32_t pin_power_map[] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6};
void process_command()
{
	if((host_receiver_buffer[0] == 0xaa) && (host_receiver_buffer[1] == 0x55) && (host_receiver_buffer[5] == 0xff))
						{
							switch(host_receiver_buffer[2])
							{
								case HOST_COMMAND_HEART_BEAT:
									usart_transmit(USART0,host_reply_heart_beat_buffer,sizeof(host_reply_heart_beat_buffer));
								break;
								case HOST_COMMAND_GET_POWER_STATUS:
									if((host_receiver_buffer[3] < 5) && (host_receiver_buffer[3] >= 0))
									{
										host_reply_get_power_status_buffer[3] = host_receiver_buffer[3];
										host_reply_get_power_status_buffer[4] = gpio_output_bit_get(GPIOA,pin_power_map[host_receiver_buffer[3]]);
										usart_transmit(USART0,host_reply_get_power_status_buffer,sizeof(host_reply_get_power_status_buffer));
									}else{usart_transmit(USART0,host_reply_error_status_buffer,sizeof(host_reply_error_status_buffer));}
								break;
								case HOST_COMMAND_SET_POWER_STATUS:
									if((host_receiver_buffer[3] < 5) && (host_receiver_buffer[3] >= 0))
									{
										gpio_bit_write(GPIOA,pin_power_map[host_receiver_buffer[3]],host_receiver_buffer[4]>0?SET:RESET);
										host_reply_set_power_status_buffer[3] = host_receiver_buffer[3];
										host_reply_set_power_status_buffer[4] = gpio_output_bit_get(GPIOA,pin_power_map[host_receiver_buffer[3]]);
										usart_transmit(USART0,host_reply_set_power_status_buffer,sizeof(host_reply_set_power_status_buffer));
									}else{usart_transmit(USART0,host_reply_error_status_buffer,sizeof(host_reply_error_status_buffer));}
								break;
								case HOST_COMMAND_GET_ADC_VALUE:
									memcpy(&host_reply_get_adc_value_buffer[3],adc_value,20);
									usart_transmit(USART0,host_reply_get_adc_value_buffer,sizeof(host_reply_get_adc_value_buffer));
								break;
								case HOST_COMMAND_GET_BAT_CAPACITY:
									battery_temp_data = BQ40Z50_Read_SOC();
									host_reply_get_bat_capacity_buffer[3] = (battery_temp_data >> 8)& 0xff;
									host_reply_get_bat_capacity_buffer[4] = battery_temp_data & 0xff;
									usart_transmit(USART0,host_reply_get_bat_capacity_buffer,sizeof(host_reply_get_bat_capacity_buffer));
								break;
								case HOST_COMMAND_GET_BAT_TEMPEARATURE:
									battery_temp_data = BQ40Z50_Read_Temp();
									host_reply_get_bat_temperature_buffer[3] = (battery_temp_data >> 8)& 0xff;
									host_reply_get_bat_temperature_buffer[4] = battery_temp_data & 0xff;
									usart_transmit(USART0,host_reply_get_bat_temperature_buffer,sizeof(host_reply_get_bat_temperature_buffer));
								break;
								case HOST_COMMAND_GET_BAT_VOLTAGE:
									battery_temp_data = BQ40Z50_Read_Vol();
									host_reply_get_bat_voltage_buffer[3] = (battery_temp_data >> 8)& 0xff;
									host_reply_get_bat_voltage_buffer[4] = battery_temp_data & 0xff;
									usart_transmit(USART0,host_reply_get_bat_voltage_buffer,sizeof(host_reply_get_bat_voltage_buffer));
								break;
								case HOST_COMMAND_BOOTUP_SUCCESS:
									host_bootup_ok = 1;
									usart_transmit(USART0,host_reply_bootup_success_buffer,sizeof(host_reply_bootup_success_buffer));
								break;								
								default:
									{usart_transmit(USART0,host_reply_error_status_buffer,sizeof(host_reply_error_status_buffer));}
								break;
							}
						}
}

#define PERIODIC_REPORT_ADC_VALUE 0x22
uint8_t periodic_report_adc_value_buffer[24] =   {0x55,0xaa,PERIODIC_REPORT_ADC_VALUE,0x00,0x00,0x00,0x00,
                                                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
																									,0x00,0x00,0x00,0x00,0x00,0x00,0xff};
void periodic_report()
{
		memcpy(&periodic_report_adc_value_buffer[3],adc_value,20);
		usart_transmit(USART0,periodic_report_adc_value_buffer,sizeof(periodic_report_adc_value_buffer));
}
uint8_t crc8(uint8_t *data, int length) {
    uint8_t crc = 0x00; // 初始值
    uint8_t poly = 0x07; // 生成多项式 x^8 + x^2 + x + 1
		int i,j;
    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) { // 如果最高位是1
                crc = (crc << 1) ^ poly; // 左移并异或生成多项式
            } else {
                crc = crc << 1; // 左移
            }
        }
    }
    return crc; // 返回最终的CRC8校验码
}

#define IIC_SDA_GPIO_Pin GPIO_PIN_9
#define IIC_SCL_GPIO_Pin GPIO_PIN_8
#define IIC_GPIOx GPIOB
#define SDA_L gpio_bit_reset(IIC_GPIOx, IIC_SDA_GPIO_Pin);
#define SDA_H gpio_bit_set(IIC_GPIOx, IIC_SDA_GPIO_Pin);
#define SCL_L gpio_bit_reset(IIC_GPIOx, IIC_SCL_GPIO_Pin);
#define SCL_H gpio_bit_set(IIC_GPIOx, IIC_SCL_GPIO_Pin);

static void Delay(int n)
{
		int target_time = 1 * n;
    while(target_time--);
}
#if 0
#if 0
void BQ40Z50_Init(void)
{
		gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,IIC_SDA_GPIO_Pin);
    gpio_output_options_set(IIC_GPIOx, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,IIC_SDA_GPIO_Pin);
	
		gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,IIC_SCL_GPIO_Pin);
    gpio_output_options_set(IIC_GPIOx, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,IIC_SCL_GPIO_Pin);
}
#else
void BQ40Z50_Init(void)
{
		rcu_periph_clock_enable(RCU_I2C0);
	  /* I2C GPIO ports */
    /* connect I2C_SCL_GPIO_PIN to I2C_SCL */
    gpio_af_set(GPIOB, GPIO_AF_1, IIC_SCL_GPIO_Pin);
    /* connect I2C_SDA_GPIO_PIN to I2C_SDA */
    gpio_af_set(GPIOB, GPIO_AF_1, IIC_SDA_GPIO_Pin);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, IIC_SCL_GPIO_Pin);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IIC_SCL_GPIO_Pin);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, IIC_SDA_GPIO_Pin);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IIC_SDA_GPIO_Pin);

	    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x72);
    /* enable I2C */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}
#endif
unsigned char SDA_READ(void)
{
	return gpio_input_bit_get(IIC_GPIOx,IIC_SDA_GPIO_Pin);
}

void IIC_Start(void)
{
	SDA_H;
	SCL_H;
	Delay(4);
	SDA_L;
	Delay(4);
	SCL_L;
	Delay(4);
}
void IIC_Stop(void)
{
	SCL_L;
	SDA_L;
	Delay(4);
	SCL_H;
	Delay(4);
	SDA_H;
	Delay(4);
}

unsigned char IIC_Wait_Ack(void)
{
	int errTime = 0;
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_INPUT, GPIO_PUPD_NONE,IIC_SDA_GPIO_Pin);
	SCL_H;
	Delay(4);
	while(SDA_READ())
	{
		errTime++;
		if(errTime >250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;
	Delay(5);
	return 0;
}
void IIC_Ack(void)
{
	SCL_L;
	SDA_L;
	Delay(4);
	SCL_H;
	Delay(4);
	SCL_L;
	Delay(4);
}

void IIC_NAck(void)
{
	SCL_L;
	Delay(4);
	SDA_H;
	Delay(4);
	SCL_H;
	Delay(4);
	SCL_L;
	Delay(4);
}

void IIC_Send_Byte(unsigned char txd)
{
	unsigned char i;
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,IIC_SDA_GPIO_Pin);
	SCL_L;
	Delay(5);
	for(i = 0;i < 8;i++)
	{
		if((txd & 0x80) >> 7)
		{SDA_H;}
		else
		{SDA_L;}
		txd <<=1;
		Delay(4);
		SCL_H;
		Delay(4);
		SCL_L;
		Delay(4);
	}
}
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,res = 0;
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_INPUT, GPIO_PUPD_NONE,IIC_SDA_GPIO_Pin);
	Delay(5);
	for(i = 0; i < 8;i++)
	{
		SCL_L;
		Delay(4);
		SCL_H;
		Delay(4);
		res <<= 1;
		if(SDA_READ())
		{
			res ++;
			Delay(4);
		}
	}
	return res;
}
#if 0
void BQ40Z25_Read_SOC(void)
{
	IIC_Start();
	IIC_Send_Byte(0x16);//写入
	if(IIC_Wait_Ack()){
		printf("device error0\r\n");
		return ;
	}
	IIC_Send_Byte(0x0d);
	if(IIC_Wait_Ack()){
		printf("device error1\r\n");
		return ;
	}
	Delay(700);
	IIC_Start();
	IIC_Send_Byte(0x17);//读取
	if(IIC_Wait_Ack()){
		printf("device error2\r\n");
		return ;
	}
	Delay(70);
	soc0 = IIC_Read_Byte(1);
	IIC_Ack();
	Delay(120);
	soc1 = IIC_Read_Byte(1);
	IIC_NAck();
	IIC_Stop();
	Delay(200);
	printf("capacity = %d\n",((soc1<<8)&0xff00)+soc0);
}
#else
void BQ40Z25_Read_SOC(void)
{
	  int i=0;
		uint8_t BQ40Z25_Battery_Capacity_address = 0x0D;
		uint8_t BQ40Z25_Battery_receive_buffer[2];
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
printf("1\r\n");
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
printf("2\r\n");
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2C0, 0x16, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
printf("3\r\n");
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
printf("4\r\n");
        /* send a data byte */
        i2c_data_transmit(I2C0,BQ40Z25_Battery_Capacity_address);
        /* wait until the transmission data register is empty*/
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
        /* wait until the RBNE bit is set */
        while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
        /* read a data from I2C_DATA */
        BQ40Z25_Battery_receive_buffer[0] = i2c_data_receive(I2C0);
				/* wait until the RBNE bit is set */
        while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
				BQ40Z25_Battery_receive_buffer[1] = i2c_data_receive(I2C0);
printf("5\r\n");
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2C0);
    while(I2C_CTL0(I2C0)&0x0200);
    while(!i2c_flag_get(I2C0, I2C_FLAG_STPDET));
    /* clear the STPDET bit */
		printf("6\r\n");
    i2c_enable(I2C0);
		BQ40Z25_Battery_receive_buffer[0] = soc0;
		BQ40Z25_Battery_receive_buffer[1] = soc1;
   printf("capacity = %d\n",((soc1<<8)&0xff00)+soc0);

}
#endif
#endif
#if 0
void i2c_sda_set(int val)   // 1 = high, 0 = low
{
	if(val)
	gpio_bit_set(IIC_GPIOx, IIC_SDA_GPIO_Pin);
	else
	gpio_bit_reset(IIC_GPIOx, IIC_SDA_GPIO_Pin);	
}

void i2c_scl_set(int val)
{
	if(val)
		gpio_bit_set(IIC_GPIOx, IIC_SCL_GPIO_Pin);
	else
		gpio_bit_reset(IIC_GPIOx, IIC_SCL_GPIO_Pin);
}

int  i2c_sda_get()	// 读取 SDA 电平
{
	int value ;
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,IIC_SDA_GPIO_Pin);
	value = gpio_input_bit_get(IIC_GPIOx,IIC_SDA_GPIO_Pin);
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,IIC_SDA_GPIO_Pin);
	return value;
}
int  i2c_scl_get()	// 读取 SDA 电平
{
	int value ;
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,IIC_SCL_GPIO_Pin);
	value = gpio_input_bit_get(IIC_GPIOx,IIC_SCL_GPIO_Pin);
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,IIC_SCL_GPIO_Pin);
	return value;
}
void i2c_delay(void)                 // 简单延时
{
	Delay(1);
}
void i2c_start(void)
{
	i2c_sda_set(1);
	i2c_scl_set(1);
	i2c_delay();
	i2c_sda_set(0);
	i2c_delay();
	i2c_scl_set(0);
}

void i2c_stop(void)
{
	i2c_sda_set(0);
	i2c_scl_set(1);
	i2c_delay();
	i2c_sda_set(1);
	i2c_delay();
}

void i2c_send_ack(int ack)
{
	i2c_sda_set(ack ? 0 : 1);
	i2c_delay();
	i2c_scl_set(1);
	i2c_delay();
	i2c_scl_set(0);
	i2c_sda_set(1); // 释放 SDA
}

int i2c_wait_ack(void)
{
	int ack;

	// 显式释放 SDA：设置为输入（高阻），而不是 i2c_sda_set(1)
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IIC_SDA_GPIO_Pin);

	i2c_delay();
	i2c_scl_set(1);
	i2c_delay();

	ack = gpio_input_bit_get(IIC_GPIOx, IIC_SDA_GPIO_Pin);  // 直接读取

	i2c_scl_set(0);
	i2c_delay();

	// 恢复 SDA 为输出模式，准备后续发送
	gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, IIC_SDA_GPIO_Pin);

	return ack == 0;
}
void i2c_write_byte(uint8_t byte)
{
	uint8_t i = 0;
	for (i = 0; i < 8; i++) {
		i2c_sda_set((byte & 0x80) != 0);
		byte <<= 1;
		i2c_delay();
		i2c_scl_set(1);
		i2c_delay();
		i2c_scl_set(0);
	}
	i2c_delay();
	i2c_sda_set(1);
	i2c_scl_set(1);
}

uint8_t i2c_read_byte(int ack)
{
    uint8_t byte = 0,i;

    // Step 1: 设置 SDA 为输入 → 释放总线
    gpio_mode_set(IIC_GPIOx, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IIC_SDA_GPIO_Pin);
		i2c_delay();
    for (i = 0; i < 8; i++) {
        byte <<= 1;

        i2c_scl_set(1);
        i2c_delay();

        if (gpio_input_bit_get(IIC_GPIOx, IIC_SDA_GPIO_Pin))
            byte |= 0x01;

        i2c_scl_set(0);
        i2c_delay();
    }

    // Step 2: 设置 SDA 为输出 → 主机发 ACK/NACK
    gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, IIC_SDA_GPIO_Pin);
    i2c_sda_set(ack ? 0 : 1);  // ACK=0继续读，NACK=1结束读

    i2c_delay();
    i2c_scl_set(1);
    i2c_delay();
    i2c_scl_set(0);

    i2c_sda_set(1);  // 释放 SDA
    return byte;
}


int read_register(uint8_t addr, uint8_t reg_no)
{
	uint8_t msb, lsb;

	i2c_start();
	i2c_write_byte((addr << 1) | 0);  // 写地址
	if (!i2c_wait_ack()) goto error;
	printf("see000\r\n");
	i2c_write_byte(reg_no);  // 写寄存器地址
	if (!i2c_wait_ack()) goto error;
	printf("see111\r\n");
	i2c_start();
	i2c_write_byte((addr << 1) | 1);  // 读地址
	if (!i2c_wait_ack()) goto error;
	printf("see222\r\n");
	msb = i2c_read_byte(1);  // ACK
	lsb = i2c_read_byte(0);  // NACK
	i2c_stop();

	return (msb << 8) | lsb;

error:
	i2c_stop();
	return -1;  // 读取失败
}

void BQ40Z50_Init(void)
{
		gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,IIC_SDA_GPIO_Pin);
    gpio_output_options_set(IIC_GPIOx, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,IIC_SDA_GPIO_Pin);
	
		gpio_mode_set(IIC_GPIOx, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,IIC_SCL_GPIO_Pin);
    gpio_output_options_set(IIC_GPIOx, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,IIC_SCL_GPIO_Pin);
}
	void BQ40Z25_Read_SOC()
{
	printf("battery capacity %d\r\n",read_register(0xB, 0xD));
	
}
#endif
/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C clock */
    rcu_periph_clock_enable(RCU_I2C0);
}

/*!
    \brief      cofigure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
#define I2C_SCL_GPIO_PIN GPIO_PIN_8
#define I2C_SDA_GPIO_PIN GPIO_PIN_9
void gpio_config(void)
{
    /* I2C GPIO ports */
    /* connect I2C_SCL_GPIO_PIN to I2C_SCL */
    gpio_af_set(GPIOB, GPIO_AF_1, I2C_SCL_GPIO_PIN);
    /* connect I2C_SDA_GPIO_PIN to I2C_SDA */
    gpio_af_set(GPIOB, GPIO_AF_1, I2C_SDA_GPIO_PIN);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_GPIO_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_GPIO_PIN);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SDA_GPIO_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_GPIO_PIN);
}

/*!
    \brief      cofigure the I2C interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void)
{
    i2c_deinit(I2C0);  // ✅ 强烈建议：先复位 I2C，清除潜在挂死状态

    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);  // 建议用 100kHz 测试更稳定

    // ✅ 主模式下，不需要设置自身地址（这步反而干扰）
    // i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x72); <-- 删除！

    i2c_enable(I2C0);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	
	
}
void i2c_scan(uint32_t i2c_bus)
{
		uint8_t addr;int timeout = 1000;
    printf("Scanning I2C bus...\r\n");
		
    for (addr = 0; addr < 0x80; addr++) {
        // 1. Start
			timeout = 1000;
        while(i2c_flag_get(i2c_bus, I2C_FLAG_I2CBSY));
        i2c_start_on_bus(i2c_bus);
        while(!i2c_flag_get(i2c_bus, I2C_FLAG_SBSEND));
        // 2. Send address
        i2c_master_addressing(i2c_bus, addr, I2C_TRANSMITTER);
        // 3. 等待ACK
        
        while (!i2c_flag_get(i2c_bus, I2C_FLAG_ADDSEND) && --timeout);
        if (timeout > 0) {
            printf("Found device at 0x%02X\r\n", addr);
            i2c_flag_clear(i2c_bus, I2C_STAT0_ADDSEND);
        }else{
					printf("not Found device at 0x%02X\r\n", addr);
				}
        // 4. Stop
        i2c_stop_on_bus(i2c_bus);
        while(I2C_CTL0(i2c_bus) & I2C_CTL0_STOP);
    }

    printf("Scan complete\r\n");
}
void BQ40Z50_Init(void)
{
    rcu_config();     // 1. 时钟先开
    gpio_config();    // 2. 再配置 GPIO 复用和开漏
    i2c_config();     // 3. 最后初始化 I2C（复位 + 启动）
}
int i2c_read_register2(uint32_t i2c_bus, uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t msb = 0, lsb = 0;
    int timeout = 10000;
		uint16_t result;
   // printf("[I2C] START READ REGISTER at device 0x%02X, reg 0x%02X\r\n", i2c_addr, reg_addr);

    /* STEP 1: Wait until bus is idle */
    timeout = 10000;
    while (i2c_flag_get(i2c_bus, I2C_FLAG_I2CBSY) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] BUS BUSY timeout\r\n");
        goto error;
    }

    /* STEP 2: Send START */
    i2c_start_on_bus(i2c_bus);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_SBSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] START condition timeout\r\n");
        goto error;
    }

    /* STEP 3: Send slave address with write bit */
    i2c_master_addressing(i2c_bus, i2c_addr, I2C_TRANSMITTER);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_ADDSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] WRITE ADDR NACK (0x%02X)\r\n", i2c_addr);
        goto error;
    }
    i2c_flag_clear(i2c_bus, I2C_STAT0_ADDSEND);

    /* STEP 4: Send register address */
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_TBE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] TBE timeout before sending reg\r\n");
        goto error;
    }
    i2c_data_transmit(i2c_bus, reg_addr);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_TBE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] TBE timeout after sending reg\r\n");
        goto error;
    }

    /* STEP 5: Repeated START */
    i2c_start_on_bus(i2c_bus);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_SBSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] Repeated START timeout\r\n");
        goto error;
    }

    /* STEP 6: Send slave address with read bit */
    i2c_master_addressing(i2c_bus, i2c_addr, I2C_RECEIVER);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_ADDSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] READ ADDR NACK (0x%02X)\r\n", i2c_addr);
        goto error;
    }
    i2c_flag_clear(i2c_bus, I2C_STAT0_ADDSEND);

    /* STEP 7: Enable ACK (for first byte) */
    i2c_ack_config(i2c_bus, I2C_ACK_ENABLE);

    /* STEP 8: Wait RBNE then read LSB (first byte) */
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_RBNE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] RBNE timeout (1st byte)\r\n");
        goto error;
    }
    lsb = i2c_data_receive(i2c_bus);

    /* STEP 9: Disable ACK, send STOP before reading second byte */
    i2c_ack_config(i2c_bus, I2C_ACK_DISABLE);
    i2c_stop_on_bus(i2c_bus);

    /* STEP 10: Wait RBNE then read MSB (second byte) */
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_RBNE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] RBNE timeout (2nd byte)\r\n");
        goto error;
    }
    msb = i2c_data_receive(i2c_bus);

    /* STEP 11: Wait for STOP to complete */
    timeout = 10000;
    while ((I2C_CTL0(i2c_bus) & I2C_CTL0_STOP) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] STOP timeout\r\n");
        goto error;
    }

    /* STEP 12: Restore ACK config */
    i2c_ack_config(i2c_bus, I2C_ACK_ENABLE);
    i2c_ackpos_config(i2c_bus, I2C_ACKPOS_CURRENT);

    result = (msb << 8) | lsb;  // ⚠️ 如果设备是低位先传，就改成 (lsb << 8) | msb
    //printf("[I2C] READ OK: 0x%04X\r\n", result);
    return result;

error:
    /* emergency recovery */
    i2c_stop_on_bus(i2c_bus);
    timeout = 10000;
    while ((I2C_CTL0(i2c_bus) & I2C_CTL0_STOP) && --timeout);
    i2c_ack_config(i2c_bus, I2C_ACK_ENABLE);
    i2c_ackpos_config(i2c_bus, I2C_ACKPOS_CURRENT);
    printf("[I2C] READ FAILED at reg 0x%02X\r\n", reg_addr);
    return -1;
}

int BQ40Z50_Read_SOC()
{
    int attempt = 0;
    int soc_raw = -1;
		uint32_t i2c_bus = I2C0;
		uint8_t i2c_addr = 0x16;
    for (attempt = 1; attempt <= 10; attempt++) {
    //    printf("[SOC] Attempt %d...\r\n", attempt);
        
        soc_raw = i2c_read_register2(i2c_bus, i2c_addr, 0x0D);  // 0x0D 是 SOC 寄存器

        if (soc_raw >= 0 && soc_raw <= 100) {
        //    printf("[SOC] Success: %d%%\r\n", soc_raw);
            return soc_raw;
        } else {
            printf("[SOC] Invalid: %d, retrying...\r\n", soc_raw);
        }
    }

    printf("[SOC] Failed after 10 attempts\r\n");
    return -1;
}

int BQ40Z50_Read_Temp()
{
    int attempt = 0;
    int soc_raw = -1;
		uint32_t i2c_bus = I2C0;
		uint8_t i2c_addr = 0x16;
    for (attempt = 1; attempt <= 10; attempt++) {
    //    printf("[SOC] Attempt %d...\r\n", attempt);
        
        soc_raw = i2c_read_register2(i2c_bus, i2c_addr, 0x08);  // 0x0D 是 温度 寄存器

        if (soc_raw > 0 ) {
        //    printf("[SOC] Success: %d%%\r\n", soc_raw);
            return soc_raw;
        } else {
            printf("[SOC] Invalid: %d, retrying...\r\n", soc_raw);
        }
    }

    printf("[SOC] Failed after 10 attempts\r\n");
    return -1;
}

int BQ40Z50_Read_Vol()
{
    int attempt = 0;
    int soc_raw = -1;
		uint32_t i2c_bus = I2C0;
		uint8_t i2c_addr = 0x16;
    for (attempt = 1; attempt <= 10; attempt++) {
    //    printf("[SOC] Attempt %d...\r\n", attempt);
        
        soc_raw = i2c_read_register2(i2c_bus, i2c_addr, 0x09);  // 0x0D 是 SOC 寄存器

        if (soc_raw > 0 && soc_raw < 65536) {
        //    printf("[SOC] Success: %d%%\r\n", soc_raw);
            return soc_raw;
        } else {
            printf("[SOC] Invalid: %d, retrying...\r\n", soc_raw);
        }
    }

    printf("[SOC] Failed after 10 attempts\r\n");
    return -1;
}