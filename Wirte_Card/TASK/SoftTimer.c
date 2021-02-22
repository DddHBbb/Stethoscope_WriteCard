#include "SoftTimer.h"
#include "rtthread.h"
#include "board.h"
#include "st25r95_com.h"

#define TIMEOVER (6 * 3)
/***********************����������*******************************/
static void LowPWR_timer_callback(void *parameter);
void LOWPWR_Config(void);
void SYSCLKConfig_STOP(void);
/***********************����������*******************************/
extern rt_mailbox_t LOW_PWR_mb;
/***********************ȫ�ֱ�����*******************************/

rt_timer_t LowPWR_timer = RT_NULL;
/****************************************************************/
static void LowPWR_timer_callback(void *parameter)
{
    static uint8_t count = 0;
    if ((rt_mb_recv(LOW_PWR_mb, RT_NULL, RT_WAITING_NO)) != RT_EOK)
    {
        count++;
        if (count == TIMEOVER)
        {
            LOWPWR_Config();
            count = 0;
        }
    }
    else
        count = 0;
}
void Timer_Init(void)
{
    LowPWR_timer = rt_timer_create("LowPWR_timer", LowPWR_timer_callback, 0, 10000, RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(LowPWR_timer);
}
void LOWPWR_Config(void)
{
    rt_kprintf("����͹���\n");
    OLED_Clear();
    Show_String(32, 12, (uint8_t *)"˯��ģʽ");
    Show_String(16, 36, (uint8_t *)"�����ؼ��˳�");
    OLED_Refresh_Gram();
    SysTick->CTRL = 0x00; //�رն�ʱ��
    SysTick->VAL = 0x00;  //���val,��ն�ʱ��
    HAL_PWREx_EnableFlashPowerDown();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    Stm32_Clock_Init(384, 25, 2, 8);
    OLED_Clear();
    Show_String(0, 0, (uint8_t *)"����״̬��");
    Show_String(32, 32, (uint8_t *)"ֹͣ����");
		BattChek();
    OLED_Refresh_Gram();
    rt_kprintf("�˳��͹���\n");
}

void SYSCLKConfig_STOP(void)
{
    /* ʹ�� HSE */
    __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);

    /* �ȴ� HSE ׼������ */
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET);

    /* ʹ�� PLL */
    __HAL_RCC_PLL_ENABLE();

    /* �ȴ� PLL ׼������ */
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* ѡ��PLL��Ϊϵͳʱ��Դ */
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);

    /* �ȴ�PLL��ѡ��Ϊϵͳʱ��Դ */
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != 0x08)
    {
    }
}








