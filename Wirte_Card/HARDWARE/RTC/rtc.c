#include "rtc.h"
#include "rtthread.h"
#include "iwdg.h"
#include "board.h"

RTC_HandleTypeDef RTC_Handler;  //RTC���

void RTC_Init(void)
{     
  	RTC_Handler.Instance=RTC;
    RTC_Handler.Init.HourFormat=RTC_HOURFORMAT_24;//RTC����Ϊ24Сʱ��ʽ 
    RTC_Handler.Init.AsynchPrediv=0X7F;           //RTC�첽��Ƶϵ��(1~0X7F)
    RTC_Handler.Init.SynchPrediv=0XFF;            //RTCͬ����Ƶϵ��(0~7FFF)   
    RTC_Handler.Init.OutPut=RTC_OUTPUT_DISABLE;     
    RTC_Handler.Init.OutPutPolarity=RTC_OUTPUT_POLARITY_HIGH;
    RTC_Handler.Init.OutPutType=RTC_OUTPUT_TYPE_OPENDRAIN;
    if(HAL_RTC_Init(&RTC_Handler)!=HAL_OK) 
			rt_kprintf("RTCʧ��\n");	
		 __HAL_RCC_RTC_DISABLE();
}
//RTC�ײ�������ʱ������
//�˺����ᱻHAL_RTC_Init()����
//hrtc:RTC���
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    __HAL_RCC_PWR_CLK_ENABLE();//ʹ�ܵ�Դʱ��PWR
//		HAL_PWR_EnableBkUpAccess();//ȡ����������д����
    
    RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_LSI;//LSE����
    RCC_OscInitStruct.PLL.PLLState=RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState=RCC_LSE_OFF;                  //RTCʹ��LSE
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    PeriphClkInitStruct.PeriphClockSelection=RCC_PERIPHCLK_RTC;//����ΪRTC
    PeriphClkInitStruct.RTCClockSelection=RCC_RTCCLKSOURCE_LSI;//RTCʱ��ԴΪLSE
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);     
    __HAL_RCC_RTC_ENABLE();//RTCʱ��ʹ��
	
		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RTC_Handler, RTC_FLAG_WUTF);//���RTC WAKE UP�ı�־
		HAL_RTCEx_SetWakeUpTimer_IT(&RTC_Handler,600,RTC_WAKEUPCLOCK_RTCCLK_DIV16);    //������װ��ֵ��ʱ�� //1000==500ms
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn,0x09,0); //��ռ���ȼ�1,�����ȼ�2
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
//	  __HAL_RCC_RTC_DISABLE();
}
//RTC WAKE UP�жϷ�����
void RTC_WKUP_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&RTC_Handler); 
}
 
//RTC WAKE UP�жϴ���
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    rt_kprintf("RTC�ж�\n");
}




















































