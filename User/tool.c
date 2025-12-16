#include "gd32f1x0_usart.h"
#include "string.h"

/* ========================= USART 初始化 ========================= */

void com_init0(void)    /* PA9: TX, PA10: RX */
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
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    usart_enable(USART0);
}

void com_init1(void)    /* PA15: RX (USART1) */
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_15);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_15);

    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
    usart_enable(USART1);
}

/* ========================= 电源阈值定义 ========================= */

#define ADC_IN0_VCC24_MIN              3217
#define ADC_IN0_VCC24_MAX              3574
#define ADC_IN1_VCC3V3_ULP_MIN         2247
#define ADC_IN1_VCC3V3_ULP_MAX         2491
#define ADC_IN4_VBAT_MIN               2518
#define ADC_IN4_VBAT_MAX               3438
#define ADC_IN5_VCC12_SYS_MIN          3084
#define ADC_IN5_VCC12_SYS_MAX          3484
#define ADC_IN6_VCC5V0_CORE_MIN        3231
#define ADC_IN6_VCC5V0_CORE_MAX        3588
#define ADC_IN7_VCC5V0_SYS_MIN         3231
#define ADC_IN7_VCC5V0_SYS_MAX         3588

void delay(int time);
extern uint8_t adc_value[20];

/* ========================= 电源初始化 ========================= */
/*
 * PA0 : 3V3 ULP
 * PA1 : 5V0 SYS
 * PA4 : 5V0 CORE
 * PA5 : 3V3 SYS
 * PA6 : 12V SYS
 * PA7 : another power
 */
void power_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);

    /* 3v3 ulp */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_0);
    gpio_bit_reset(GPIOA, GPIO_PIN_0);

    /* 5v0 sys */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_1);
    gpio_bit_reset(GPIOA, GPIO_PIN_1);

    /* 5v0 core */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_4);
    gpio_bit_reset(GPIOA, GPIO_PIN_4);

    /* 3v3 sys */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_5);
    gpio_bit_reset(GPIOA, GPIO_PIN_5);

    /* 12v sys */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_6);
    gpio_bit_reset(GPIOA, GPIO_PIN_6);

    /* another power */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_7);
    gpio_bit_reset(GPIOA, GPIO_PIN_7);

    /* hardware define boot sequence */
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

    /* 原来 #if 0 中的 ADC 检查逻辑是关闭状态，这里保持原有行为：直接打开 5V0 CORE */
    gpio_bit_set(GPIOA, GPIO_PIN_4);
}

/* ========================= USART 发送 ========================= */

void usart_transmit(uint32_t usart_periph, uint8_t *data, uint8_t length)
{
    uint8_t i;

    for (i = 0; i < length; i++) {
        usart_data_transmit(usart_periph, *(data + i));
        while (RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
    }
}

/* ========================= Host 协议处理 ========================= */

#define HOST_RECEIVE_COUNT                 6
#define HOST_COMMAND_HEART_BEAT            0x00
#define HOST_COMMAND_GET_POWER_STATUS      0x01
#define HOST_COMMAND_SET_POWER_STATUS      0x02
#define HOST_COMMAND_GET_ADC_VALUE         0x03
#define HOST_COMMAND_GET_BAT_CAPACITY      0x04
#define HOST_COMMAND_GET_BAT_TEMPEARATURE  0x05
#define HOST_COMMAND_GET_BAT_VOLTAGE       0x06
#define HOST_COMMAND_BOOTUP_SUCCESS        0x10

extern uint8_t adc_value[];
extern uint8_t host_receiver_buffer[HOST_RECEIVE_COUNT];
extern uint8_t host_bootup_ok;

int battery_temp_data = -1;

/* 回复缓冲区 */
uint8_t host_reply_heart_beat_buffer[6]          = {0x55, 0xaa, HOST_COMMAND_HEART_BEAT,           0x00, 0x00, 0xff};
uint8_t host_reply_get_power_status_buffer[6]    = {0x55, 0xaa, HOST_COMMAND_GET_POWER_STATUS,     0x00, 0x00, 0xff};
uint8_t host_reply_set_power_status_buffer[6]    = {0x55, 0xaa, HOST_COMMAND_SET_POWER_STATUS,     0x00, 0x00, 0xff};
uint8_t host_reply_get_adc_value_buffer[24]      = {0x55, 0xaa, HOST_COMMAND_GET_ADC_VALUE,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0xff};
uint8_t host_reply_get_bat_capacity_buffer[6]    = {0x55, 0xaa, HOST_COMMAND_GET_BAT_CAPACITY,     0x00, 0x00, 0xff};
uint8_t host_reply_get_bat_temperature_buffer[6] = {0x55, 0xaa, HOST_COMMAND_GET_BAT_TEMPEARATURE, 0x00, 0x00, 0xff};
uint8_t host_reply_get_bat_voltage_buffer[6]     = {0x55, 0xaa, HOST_COMMAND_GET_BAT_VOLTAGE,      0x00, 0x00, 0xff};
uint8_t host_reply_bootup_success_buffer[6]      = {0x55, 0xaa, HOST_COMMAND_BOOTUP_SUCCESS,       0x00, 0x00, 0xff};
uint8_t host_reply_error_status_buffer[6]        = {0x55, 0xaa, 0xff, 0xff, 0xff, 0xff};

/*               3v3 ulp     5v0 sys    5v0 core    3v3 sys     12v sys  */
uint32_t pin_power_map[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6};

void process_command(void)
{
    /* 帧格式固定为: [0]=0xaa, [1]=0x55, [2]=cmd, [3],[4]=data, [5]=0xff */
    if ((host_receiver_buffer[0] == 0xaa) &&
        (host_receiver_buffer[1] == 0x55) &&
        (host_receiver_buffer[5] == 0xff)) {

        switch (host_receiver_buffer[2]) {

        case HOST_COMMAND_HEART_BEAT:
            usart_transmit(USART0,
                           host_reply_heart_beat_buffer,
                           sizeof(host_reply_heart_beat_buffer));
            break;

        case HOST_COMMAND_GET_POWER_STATUS:
            if (host_receiver_buffer[3] < 5) {
                host_reply_get_power_status_buffer[3] =
                    host_receiver_buffer[3];
                host_reply_get_power_status_buffer[4] =
                    gpio_output_bit_get(GPIOA,
                                        pin_power_map[host_receiver_buffer[3]]);
                usart_transmit(USART0,
                               host_reply_get_power_status_buffer,
                               sizeof(host_reply_get_power_status_buffer));
            } else {
                usart_transmit(USART0,
                               host_reply_error_status_buffer,
                               sizeof(host_reply_error_status_buffer));
            }
            break;

        case HOST_COMMAND_SET_POWER_STATUS:
            if (host_receiver_buffer[3] < 5) {
                gpio_bit_write(GPIOA,
                               pin_power_map[host_receiver_buffer[3]],
                               (host_receiver_buffer[4] > 0) ? SET : RESET);
                host_reply_set_power_status_buffer[3] =
                    host_receiver_buffer[3];
                host_reply_set_power_status_buffer[4] =
                    gpio_output_bit_get(GPIOA,
                                        pin_power_map[host_receiver_buffer[3]]);
                usart_transmit(USART0,
                               host_reply_set_power_status_buffer,
                               sizeof(host_reply_set_power_status_buffer));
            } else {
                usart_transmit(USART0,
                               host_reply_error_status_buffer,
                               sizeof(host_reply_error_status_buffer));
            }
            break;

        case HOST_COMMAND_GET_ADC_VALUE:
            memcpy(&host_reply_get_adc_value_buffer[3], adc_value, 20);
            usart_transmit(USART0,
                           host_reply_get_adc_value_buffer,
                           sizeof(host_reply_get_adc_value_buffer));
            break;

        case HOST_COMMAND_GET_BAT_CAPACITY:
            battery_temp_data = BQ40Z50_Read_SOC();
            host_reply_get_bat_capacity_buffer[3] =
                (battery_temp_data >> 8) & 0xff;
            host_reply_get_bat_capacity_buffer[4] =
                battery_temp_data & 0xff;
            usart_transmit(USART0,
                           host_reply_get_bat_capacity_buffer,
                           sizeof(host_reply_get_bat_capacity_buffer));
            break;

        case HOST_COMMAND_GET_BAT_TEMPEARATURE:
            battery_temp_data = BQ40Z50_Read_Temp();
            host_reply_get_bat_temperature_buffer[3] =
                (battery_temp_data >> 8) & 0xff;
            host_reply_get_bat_temperature_buffer[4] =
                battery_temp_data & 0xff;
            usart_transmit(USART0,
                           host_reply_get_bat_temperature_buffer,
                           sizeof(host_reply_get_bat_temperature_buffer));
            break;

        case HOST_COMMAND_GET_BAT_VOLTAGE:
            battery_temp_data = BQ40Z50_Read_Vol();
            host_reply_get_bat_voltage_buffer[3] =
                (battery_temp_data >> 8) & 0xff;
            host_reply_get_bat_voltage_buffer[4] =
                battery_temp_data & 0xff;
            usart_transmit(USART0,
                           host_reply_get_bat_voltage_buffer,
                           sizeof(host_reply_get_bat_voltage_buffer));
            break;

        case HOST_COMMAND_BOOTUP_SUCCESS:
            host_bootup_ok = 1;
            usart_transmit(USART0,
                           host_reply_bootup_success_buffer,
                           sizeof(host_reply_bootup_success_buffer));
            break;

        default:
            usart_transmit(USART0,
                           host_reply_error_status_buffer,
                           sizeof(host_reply_error_status_buffer));
            break;
        }
    }
}

/* ========================= 周期上报 ADC ========================= */

#define PERIODIC_REPORT_ADC_VALUE 0x22

uint8_t periodic_report_adc_value_buffer[24] = {
    0x55, 0xaa, PERIODIC_REPORT_ADC_VALUE,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xff
};

void periodic_report(void)
{
    memcpy(&periodic_report_adc_value_buffer[3], adc_value, 20);
    usart_transmit(USART0,
                   periodic_report_adc_value_buffer,
                   sizeof(periodic_report_adc_value_buffer));
}

/* ========================= CRC8 计算 ========================= */

uint8_t crc8(uint8_t *data, int length)
{
    uint8_t crc  = 0x00; /* 初始值 */
    uint8_t poly = 0x07; /* 生成多项式 x^8 + x^2 + x + 1 */
    int i, j;

    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ poly);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ========================= I2C + BQ40Z50 支持（硬件 I2C） ========================= */

static void Delay(int n)
{
    int target_time = 1 * n;
    while (target_time--);
}

/*!
    \brief      enable the peripheral clock
*/
void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C clock */
    rcu_periph_clock_enable(RCU_I2C0);
}

/*!
    \brief      configure the GPIO ports for I2C
*/
#define I2C_SCL_GPIO_PIN   GPIO_PIN_8
#define I2C_SDA_GPIO_PIN   GPIO_PIN_9

void gpio_config(void)
{
    /* I2C GPIO ports: PB8(SCL), PB9(SDA) */
    gpio_af_set(GPIOB, GPIO_AF_1, I2C_SCL_GPIO_PIN);
    gpio_af_set(GPIOB, GPIO_AF_1, I2C_SDA_GPIO_PIN);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_GPIO_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_GPIO_PIN);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SDA_GPIO_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_GPIO_PIN);
}

/*!
    \brief      configure the I2C interface
*/
void i2c_config(void)
{
    /* 先复位 I2C，清除潜在挂死状态 */
    i2c_deinit(I2C0);

    /* I2C clock configure: 100kHz */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);

    /* 主模式不设置本机地址 */

    i2c_enable(I2C0);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

/* I2C 扫描总线（调试用） */
void i2c_scan(uint32_t i2c_bus)
{
    uint8_t addr;
    int timeout;

    printf("Scanning I2C bus...\r\n");

    for (addr = 0; addr < 0x80; addr++) {
        timeout = 1000;

        /* wait bus idle */
        while (i2c_flag_get(i2c_bus, I2C_FLAG_I2CBSY));

        /* start */
        i2c_start_on_bus(i2c_bus);
        while (!i2c_flag_get(i2c_bus, I2C_FLAG_SBSEND));

        /* send address */
        i2c_master_addressing(i2c_bus, addr, I2C_TRANSMITTER);

        /* wait ACK */
        while (!i2c_flag_get(i2c_bus, I2C_FLAG_ADDSEND) && --timeout);

        if (timeout > 0) {
            printf("Found device at 0x%02X\r\n", addr);
            i2c_flag_clear(i2c_bus, I2C_STAT0_ADDSEND);
        } else {
            printf("not Found device at 0x%02X\r\n", addr);
        }

        /* stop */
        i2c_stop_on_bus(i2c_bus);
        while (I2C_CTL0(i2c_bus) & I2C_CTL0_STOP);
    }

    printf("Scan complete\r\n");
}

/* BQ40Z50 初始化：时钟 -> GPIO -> I2C */
void BQ40Z50_Init(void)
{
    rcu_config();
    gpio_config();
    i2c_config();
}

/* ========= 通用 I2C 寄存器读取函数 ========= */
/* 读取 16bit 寄存器（高字节在前：MSB:高字节, LSB:低字节），保持与原先组合方式一致：result = (msb<<8)|lsb */
int i2c_read_register2(uint32_t i2c_bus, uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t msb = 0, lsb = 0;
    int timeout;
    uint16_t result;

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

    /* 按原逻辑：高字节在前 */
    result = (uint16_t)((msb << 8) | lsb);
    return (int)result;

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

/* BQ40Z50 器件地址、总线常量 */
#define BQ40Z50_I2C_BUS      I2C0
#define BQ40Z50_I2C_ADDR     0x16  /* 与原代码一致 */

/* ========= BQ40Z50 寄存器封装 ========= */
/* 注意：这里只做“读取原始寄存器值”的封装，不做物理单位换算，以保证与原逻辑一致 */

int BQ40Z50_Read_SOC(void)          /* 原有：电量百分比，寄存器 0x0D */
{
    int attempt;
    int soc_raw = -1;

    for (attempt = 1; attempt <= 10; attempt++) {
        soc_raw = i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0D);

        if (soc_raw >= 0 && soc_raw <= 100) {
            return soc_raw;
        } else {
            printf("[SOC] Invalid: %d, retrying...\r\n", soc_raw);
        }
    }

    printf("[SOC] Failed after 10 attempts\r\n");
    return -1;
}

/* 电池温度：0x08（原有函数保持） */
int BQ40Z50_Read_Temp(void)
{
    int attempt;
    int temp_raw = -1;

    for (attempt = 1; attempt <= 10; attempt++) {
        temp_raw = i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x08);

        if (temp_raw > 0) {
            return temp_raw;
        } else {
            printf("[TEMP] Invalid: %d, retrying...\r\n", temp_raw);
        }
    }

    printf("[TEMP] Failed after 10 attempts\r\n");
    return -1;
}

/* 电池电压：0x09（原有函数保持） */
int BQ40Z50_Read_Vol(void)
{
    int attempt;
    int vol_raw = -1;

    for (attempt = 1; attempt <= 10; attempt++) {
        vol_raw = i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x09);

        if (vol_raw > 0 && vol_raw < 65536) {
            return vol_raw;
        } else {
            printf("[VOL] Invalid: %d, retrying...\r\n", vol_raw);
        }
    }

    printf("[VOL] Failed after 10 attempts\r\n");
    return -1;
}

/* ========= 你列出的其它寄存器，分别封装为独立函数 ========= */

/* 充电次数（Cycle Count）：寄存器 0x17 */
int BQ40Z50_Read_CycleCount(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x17);
}

/* 电池满电容量（Full Charge Capacity）：寄存器 0x10 */
int BQ40Z50_Read_FullChargeCapacity(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x10);
}

/* 电池设计容量（Design Capacity）：寄存器 0x18 */
int BQ40Z50_Read_DesignCapacity(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x18);
}

/* 剩余容量（Remaining Capacity）：寄存器 0x0F */
int BQ40Z50_Read_RemainingCapacity(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0F);
}

/* 是否在充电（Charging Status 等）：寄存器 0x16
 * 具体 bit 含义由上层解析，这里只返回原始 16bit 数据
 */
int BQ40Z50_Read_ChargingStatus(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x16);
}

/* 预计充满时间（Time to Full）：寄存器 0x13 */
int BQ40Z50_Read_TimeToFull(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x13);
}

/* 电池电流（Current）：寄存器 0x0A */
int BQ40Z50_Read_Current(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0A);
}

/* ========================= 周期上报（新协议 AA AA AA） ========================= */
/*
 * 协议格式：
 *   Header : 0xAA 0xAA 0xAA
 *   Len    : 2 字节，大端，高字节在前，值 = Type(1) + DataLen(1+N) + Len字段本身(2)
 *   Type   : 0xFE
 *   Data   : [SubCmd(1)] + Payload(N)
 *   CRC    : 1 字节，普通累加取低 8 位（从 Len 高字节到 Data 最后一字节）
 *   Tail   : 0xFF 0xFF 0xFF
 */

extern uint8_t adc_value[20];

/* ----------- 普通累加 CRC 计算（仅用于新协议） ----------- */
static uint8_t calc_add_crc(uint8_t *data, uint16_t length)
{
    uint32_t sum = 0;
    uint16_t i;

    for (i = 0; i < length; i++) {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFF);
}

/* ----------- 20 路 ADC 周期上报，子命令 0x80 ----------- */
/* 数据体 = [0x80] + 20 字节 adc_value */
void periodic_adc_report(void)
{
    uint8_t buf[64];  /* 足够容纳本帧 */
    uint16_t idx = 0;
    uint16_t len_field;      /* 协议中的长度值 */
    uint8_t  len_hi, len_lo;
    uint8_t  crc;
    uint16_t crc_start_index;
    uint16_t crc_len;
    uint8_t  i;

    /* 有效数据长度（子命令 + 20 字节 ADC） */
    const uint16_t payload_len = 1 + 20;
    /* Type(1) + Data(1+20) + Len(2) */
    len_field = 1 + payload_len + 2;
    len_hi    = (uint8_t)((len_field >> 8) & 0xFF);
    len_lo    = (uint8_t)(len_field & 0xFF);

    /* Header */
    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;

    /* Length (2B, big endian) */
    buf[idx++] = len_hi;
    buf[idx++] = len_lo;

    /* Type */
    buf[idx++] = 0xFE;

    /* Data: SubCmd */
    buf[idx++] = 0x80;

    /* Data: Payload (20 bytes ADC) */
    for (i = 0; i < 20; i++) {
        buf[idx++] = adc_value[i];
    }

    /* 计算 CRC：从 len_hi 开始到最后一个数据字节 */
    crc_start_index = 3; /* buf[3] = len_hi */
    crc_len         = idx - crc_start_index;
    crc             = calc_add_crc(&buf[crc_start_index], crc_len);

    /* CRC */
    buf[idx++] = crc;

    /* Tail */
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;

    /* 通过 USART0 发送 */
    usart_transmit(USART0, buf, (uint8_t)idx);
}

/* ----------- 电池信息周期上报，子命令 0x81 ----------- */
/*
 * 需要实时读取 9 个寄存器，每个 2 字节（大端在前）：
 *   1) 0x17 Cycle Count
 *   2) 0x10 Full Charge Capacity
 *   3) 0x18 Design Capacity
 *   4) 0x0F Remaining Capacity
 *   5) 0x16 Charging Status
 *   6) 0x13 Time to Full
 *   7) 0x08 Temp
 *   8) 0x09 Voltage
 *   9) 0x0A Current
 * 共 9 * 2 = 18 字节，有效数据 = [子命令(1)] + 18 字节 = 19 字节
 */

int BQ40Z50_Read_CycleCount(void);
int BQ40Z50_Read_FullChargeCapacity(void);
int BQ40Z50_Read_DesignCapacity(void);
int BQ40Z50_Read_RemainingCapacity(void);
int BQ40Z50_Read_ChargingStatus(void);
int BQ40Z50_Read_TimeToFull(void);
int BQ40Z50_Read_Temp(void);
int BQ40Z50_Read_Vol(void);
int BQ40Z50_Read_Current(void);

void periodic_battery_report(void)
{
    uint8_t buf[96];   /* 足够容纳本帧 */
    uint16_t idx = 0;
    uint16_t len_field;
    uint8_t  len_hi, len_lo;
    uint8_t  crc;
    uint16_t crc_start_index;
    uint16_t crc_len;
    uint16_t chg_raw = 0;

    /* 9 个 16bit 数，按大端顺序输出 */
    int val_cycle        = BQ40Z50_Read_CycleCount();
    int val_full_cap     = BQ40Z50_Read_FullChargeCapacity();
    int val_design_cap   = BQ40Z50_Read_DesignCapacity();
    int val_remaining    = BQ40Z50_Read_RemainingCapacity();
    int val_chg_status   = BQ40Z50_Read_ChargingStatus();
    int val_time_to_full = BQ40Z50_Read_TimeToFull();
    int val_temp         = BQ40Z50_Read_Temp();
    int val_vol          = BQ40Z50_Read_Vol();
    int val_curr         = BQ40Z50_Read_Current();

    /* ----------------- 根据 0x16 充电状态控制电池灯 ----------------- */
    /* 静态变量记录上一次灯的开关状态，用于闪烁 */
    static uint8_t s_bat_led_on = 0;

    /* 这里示例：
     *   假设 val_chg_status 的 bit4 表示“正在充电”，bit5 表示“已充满”。
     *   你可以按实际 BQ40Z50 的状态位定义修改下面两个宏。
     */
    #define CHG_STATUS_CHARGING_MASK   (1 << 4)
    #define CHG_STATUS_FULL_MASK       (1 << 5)

    if (val_chg_status >= 0) {
        chg_raw = (uint16_t)val_chg_status;

        if (chg_raw & CHG_STATUS_CHARGING_MASK) {
            /* 充电中：闪烁灯 */
            s_bat_led_on = !s_bat_led_on;
            if (s_bat_led_on) {
                gpio_bit_set(GPIOC, GPIO_PIN_13);   /* 灯亮 */
            } else {
                gpio_bit_reset(GPIOC, GPIO_PIN_13); /* 灯灭 */
            }
        } else if (chg_raw & CHG_STATUS_FULL_MASK) {
            /* 充满：常亮灯 */
            s_bat_led_on = 1;
            gpio_bit_set(GPIOC, GPIO_PIN_13);
        } else {
            /* 其它状态：灭灯，避免误亮 */
            s_bat_led_on = 0;
            gpio_bit_reset(GPIOC, GPIO_PIN_13);
        }
    } else {
        /* 读取失败时，关灯防止误判 */
        s_bat_led_on = 0;
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
    }

    /* 有效数据长度（子命令 + 9*2 字节） */
    const uint16_t payload_len = 1 + 18;
    /* Type(1) + Data(1+18) + Len(2) */
    len_field = 1 + payload_len + 2;
    len_hi    = (uint8_t)((len_field >> 8) & 0xFF);
    len_lo    = (uint8_t)(len_field & 0xFF);

    /* Header */
    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;

    /* Length (big endian) */
    buf[idx++] = len_hi;
    buf[idx++] = len_lo;

    /* Type */
    buf[idx++] = 0xFE;

    /* Data: SubCmd */
    buf[idx++] = 0x81;

    /* 一个小宏，方便以大端写入 16bit 值。
     * 如果读取失败返回 -1，则填 0xFFFF 占位。
     */
#define PUT_16BE_FROM_VAL(v)                 \
    do {                                     \
        uint16_t _u16;                       \
        if ((v) < 0) {                       \
            _u16 = 0xFFFF;                   \
        } else {                             \
            _u16 = (uint16_t)(v);            \
        }                                    \
        buf[idx++] = (uint8_t)((_u16 >> 8) & 0xFF); \
        buf[idx++] = (uint8_t)(_u16 & 0xFF);        \
    } while (0)

    PUT_16BE_FROM_VAL(val_cycle);        /* 0x17 Cycle Count */
    PUT_16BE_FROM_VAL(val_full_cap);     /* 0x10 Full Charge Capacity */
    PUT_16BE_FROM_VAL(val_design_cap);   /* 0x18 Design Capacity */
    PUT_16BE_FROM_VAL(val_remaining);    /* 0x0F Remaining Capacity */
    PUT_16BE_FROM_VAL(val_chg_status);   /* 0x16 Charging Status (原始 16bit) */
    PUT_16BE_FROM_VAL(val_time_to_full); /* 0x13 Time to Full */
    PUT_16BE_FROM_VAL(val_temp);         /* 0x08 Temperature */
    PUT_16BE_FROM_VAL(val_vol);          /* 0x09 Voltage */
    PUT_16BE_FROM_VAL(val_curr);         /* 0x0A Current */

#undef PUT_16BE_FROM_VAL

    /* 计算 CRC：从 len_hi 开始到最后一个数据字节 */
    crc_start_index = 3; /* buf[3] = len_hi */
    crc_len         = idx - crc_start_index;
    crc             = calc_add_crc(&buf[crc_start_index], crc_len);

    /* CRC */
    buf[idx++] = crc;

    /* Tail */
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;

    /* 通过 USART0 发送 */
    usart_transmit(USART0, buf, (uint8_t)idx);
}
