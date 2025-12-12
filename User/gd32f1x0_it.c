/*!
    \file  gd32f1x0_it.c
    \brief interrupt service routines
*/

/*
    Copyright (C) 2017 GigaDevice

    2014-12-26, V1.0.0, platform GD32F1x0(x=3,5)
    2016-01-15, V2.0.0, platform GD32F1x0(x=3,5,7,9)
    2016-04-30, V3.0.0, firmware update for GD32F1x0(x=3,5,7,9)
    2017-06-19, V3.1.0, firmware update for GD32F1x0(x=3,5,7,9)
*/

#include "gd32f1x0_it.h"
#include "systick.h"
#include "string.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
}
/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
#define MCU_RECEIVE_COUNT 24
__IO uint16_t usart1_rxcount; 
uint8_t usart1_receiver_buffer[32];
uint8_t adc_value[20];
extern int need_report_adc;
int message1_begin = 0;
void USART1_IRQHandler(void)//与另一个芯片进行通信的串口
{
    if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)) {
        /* 接收数据 */
        uint8_t received_byte = usart_data_receive(USART1);

        // 如果缓冲区已满，则丢弃新数据
        if (usart1_rxcount >= MCU_RECEIVE_COUNT) {
            return;
        }

        // 将数据添加到缓冲区
        usart1_receiver_buffer[usart1_rxcount++] = received_byte;

        // 检查是否开始了一个新的消息
        if (usart1_rxcount >= 1) {
            // 如果第一位不是AA，但第二位是AA，那么将第二位前移
            if (usart1_rxcount == 2) {
                if (usart1_receiver_buffer[0] != 0xAA && usart1_receiver_buffer[1] == 0xAA) {
                    memmove(usart1_receiver_buffer, &usart1_receiver_buffer[1], usart1_rxcount - 1);
                    usart1_rxcount--;
                }
            }

            // 检查是否开始了一个新的消息
            if (usart1_rxcount >= 2 && message1_begin == 0) {
                if (usart1_receiver_buffer[0] == 0xAA && usart1_receiver_buffer[1] == 0xAA) {
                    message1_begin = 1;
                } else {
                    // 如果前两位不是AA AA，则丢弃数据并重置
                    usart1_rxcount = 0;
                    message1_begin = 0;
                }
            }
        }
        // 检查是否接收到完整帧
        if (usart1_rxcount == MCU_RECEIVE_COUNT) {
            usart_interrupt_disable(USART1, USART_INT_RBNE);

            // 检查消息开始标志和数据有效性
            if (message1_begin == 1) {
                // 检查最后两个字节是否是FF
                if (usart1_receiver_buffer[MCU_RECEIVE_COUNT - 1] == 0xFF) {
                    memcpy(adc_value, &usart1_receiver_buffer[2], 20);
                    need_report_adc = 1;
                } else {
                    printf("Invalid end byte: Expected 0xFF\n");
                }
            } else {
                printf("Message begin not detected\n");
            }

            // 重置接收状态
            usart1_rxcount = 0;
            message1_begin = 0;
            usart_interrupt_enable(USART1, USART_INT_RBNE);
        }
    }
}

#define HOST_RECEIVE_COUNT 6
uint8_t host_receiver_buffer[6];
__IO uint16_t usart0_rxcount; 
uint8_t usart0_receiver_buffer[16];
extern uint8_t need_process_command;
int message0_begin = 0;
void usart_transmit(uint32_t usart_periph,uint8_t *data,uint8_t length);
void USART0_IRQHandler(void) //与主芯片进行通信的串口，负责处理协议
{
    //printf("x");
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) {
        /* 接收数据 */
        uint8_t received_byte = usart_data_receive(USART0);

        // 如果缓冲区已满，则丢弃新数据
        if (usart0_rxcount >= HOST_RECEIVE_COUNT) {
            return;
        }

        // 将数据添加到缓冲区
        usart0_receiver_buffer[usart0_rxcount++] = received_byte;

        // 如果缓冲区只有一个字节，并且不是0xAA，则清空缓冲区，直到第一个字节是0xAA
        if (usart0_rxcount == 1) {
            if (usart0_receiver_buffer[0] != 0xAA) {
                // 清空缓冲区并继续接收
                usart0_rxcount = 0;
                message0_begin = 0;
                return;
            }
        }

        // 如果缓冲区有两个字节，检查第二个字节是否为0x55
        if (usart0_rxcount == 2) {
            if (usart0_receiver_buffer[1] != 0x55) {
                // 如果第二个字节不是0x55，但第一个字节是0xAA，且第二个字节是0xAA
                if (usart0_receiver_buffer[0] == 0xAA && usart0_receiver_buffer[1] == 0xAA) {
                    // 将第二个字节移到第一个位置
                    usart0_receiver_buffer[0] = usart0_receiver_buffer[1];
                    // 计数器减一，表示还没有接收到第二个有效字节
                    usart0_rxcount = 1;
                } else {
                    // 如果不是有效的帧头，则清空缓冲区
                    usart0_rxcount = 0;
                    message0_begin = 0;
                }
                return;
            }
        }

        // 如果已经接收到完整帧，检查结束字节是否为0xFF
        if (usart0_rxcount == HOST_RECEIVE_COUNT) {
            usart_interrupt_disable(USART0, USART_INT_RBNE);  // 禁用中断

            // 检查最后一个字节是否是0xFF
            if (usart0_receiver_buffer[HOST_RECEIVE_COUNT - 1] == 0xFF) {
                // 拷贝有效数据（假设数据从第3字节开始，共6字节）
                memcpy(host_receiver_buffer, &usart0_receiver_buffer[0], 6);
                need_process_command = 1;
            } else {
                // 结束字节不为0xFF，丢弃数据并重置
                printf("Invalid end byte: Expected 0xFF\n");
            }

            // 重置接收状态
            usart0_rxcount = 0;
            message0_begin = 0;

            usart_interrupt_enable(USART0, USART_INT_RBNE);  // 重新启用中断
        }
    }
}
