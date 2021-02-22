#ifndef __BOARD_H__
#define __BOARD_H__

/*
*************************************************************************
*                             ������ͷ�ļ�
*************************************************************************
*/
#include "rtthread.h"
/* STM32 �̼���ͷ�ļ� */
#include "stm32f4xx.h"

/* ������Ӳ��bspͷ�ļ� */
#include "usart.h"
#include "sys.h"
#include "D_delay.h"
#include "usart.h"
#include "string.h"
#include "w25qxx.h"
#include "ff.h"
#include "exfuns.h"
#include "string.h"
#include "sdio_sdcard.h"
#include "fontupd.h"
#include "text.h"
#include "wm8978.h"	 
#include "wavplay.h" 
#include "GPIOConfig.h"
#include "oled.h"
#include "spi.h"
#include "iwdg.h"
#include "key.h"  
#include "low_pwr.h"
#include "rtc.h"

/*
*************************************************************************
*                               ��������
*************************************************************************
*/
void Timer_Init(void);
void Task_init(void);
void Semaphore_init(void);
void Mailbox_init(void);
void Event_init(void);
void rt_hw_us_delay(rt_uint32_t us);
char rt_hw_console_getchar(void);

#endif /* __BOARD_H__ */
