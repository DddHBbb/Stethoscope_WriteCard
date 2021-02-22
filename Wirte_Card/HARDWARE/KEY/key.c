#include "key.h"
#include "D_delay.h"

/**
  * @brief  ���ð����õ���I/O��
  * @param  ��
  * @retval ��
  */
void Key_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*��������*/
    GPIO_Initure.Pin = WAKEUP_PIN;
    GPIO_Initure.Mode = GPIO_MODE_INPUT;
    GPIO_Initure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(WAKEUP_PORT, &GPIO_Initure);
	

    GPIO_Initure.Pin = KEY_UP_PIN;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(KEY_UP_PORT, &GPIO_Initure);

    GPIO_Initure.Pin = KEY_DOWN_PIN;
    HAL_GPIO_Init(KEY_DOWN_PORT, &GPIO_Initure);

    GPIO_Initure.Pin = KEY_OK_PIN;
    HAL_GPIO_Init(KEY_OK_PORT, &GPIO_Initure);
}

uint8_t KEY_Scan(void)
{
    uint8_t key_up = 1; //�����ɿ���־

    if (key_up && (KEY0 == 1 || KEY1 == 1 || KEY2 == 1 || WK_UP == 1))
    {
				delay_ms(80);
        key_up = 0;
        if (KEY0 == 1)
            return KEY_UP;
        else if (KEY1 == 1)
            return KEY_DOWN;
        else if (KEY2 == 1)
            return KEY_OK;
        else if (WK_UP == 1)
            return WKUP_PRES;
    }
    else if (KEY0 == 0 && KEY1 == 0 && KEY2 == 0 && WK_UP == 0)
        key_up = 1;
    return 0; //�ް�������
}

//ֻ����Ϊ��������
uint8_t Key_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    static uint8_t Key_Press_Start = 0;  //�������±�־
    static uint8_t Key_Press_Stop = 0;   //���������־
    static uint16_t Key_Press_count = 0; //������������
    static uint32_t Key_Press_Times = 0; //������������
    static uint8_t Key_Press_End = 0;    //��������������־
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)

    {
        //�������µ����������������
        if (Key_Press_Start == 0)
        {
            Key_Press_Start = 1;
            Key_Press_Stop = 0;
            Key_Press_Times++; //���ذ�������
            Key_Press_count = 0;
        }
        if (Key_Press_Start == 1)
        {
            Key_Press_count++;
            if (Key_Press_count > 300) //�������³�ʱ
            {
                Key_Press_count = 0;
                Key_Press_Start = 0;
                Key_Press_Stop = 0;
            }
        }
    }
    else
    {
        //�������𵥻��������������
        if ((Key_Press_Stop == 0) && (Key_Press_Start == 1))
        {
            Key_Press_Stop = 1;
            Key_Press_Start = 0;
            Key_Press_count = 0;
            //				Key_Press_Times=0;//ֻ���ڷ���0��1������ɾ��
        }
        if (Key_Press_Stop == 1)
        {
            Key_Press_count++;
            if (Key_Press_count > 300) //��������ʱ
            {
                Key_Press_count = 0;
                Key_Press_Start = 0;
                Key_Press_Stop = 0;
                Key_Press_End = 1;
            }
        }
    }
    if (Key_Press_Times >= 4)
        Key_Press_Times = 4;

    return Key_Press_Times;
}

/*********************************************END OF FILE**********************/
