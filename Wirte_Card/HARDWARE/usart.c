#include "usart.h"
#include "logger.h"

extern rt_mailbox_t BuleTooth_Transfer_mb;
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���

struct __FILE
{
    int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    USART1->DR = (u8)ch;
    while ((USART1->SR & 0X40) == 0); //ѭ������,ֱ���������
    return ch;
}

//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
u8 USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA = 0; //����״̬���

u8 aRxBuffer[RXBUFFERSIZE]; //HAL��ʹ�õĴ��ڽ��ջ���
volatile u8 aRxBuffer1[2];
UART_HandleTypeDef UART1_Handler; //UART���
UART_HandleTypeDef UART3_Handler; //UART���

//��ʼ��IO ����1
//bound:������
void uart_init(void)
{
    //UART ��ʼ������
    UART1_Handler.Instance = USART1;                    //USART1
    UART1_Handler.Init.BaudRate = 115200;               //������
    UART1_Handler.Init.WordLength = UART_WORDLENGTH_8B; //�ֳ�Ϊ8λ���ݸ�ʽ
    UART1_Handler.Init.StopBits = UART_STOPBITS_1;      //һ��ֹͣλ
    UART1_Handler.Init.Parity = UART_PARITY_NONE;       //����żУ��λ
    UART1_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; //��Ӳ������
    UART1_Handler.Init.Mode = UART_MODE_TX_RX;          //�շ�ģʽ
    UART1_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&UART1_Handler);                                  //HAL_UART_Init()��ʹ��UART1
    HAL_UART_Receive_IT(&UART1_Handler, (uint8_t *)&aRxBuffer1, 2); //�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������

    UART3_Handler.Instance = USART3;                    //USART1
    UART3_Handler.Init.BaudRate = 115200;               //������
    UART3_Handler.Init.WordLength = UART_WORDLENGTH_8B; //�ֳ�Ϊ8λ���ݸ�ʽ
    UART3_Handler.Init.StopBits = UART_STOPBITS_1;      //һ��ֹͣλ
    UART3_Handler.Init.Parity = UART_PARITY_NONE;       //����żУ��λ
    UART3_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; //��Ӳ������
    UART3_Handler.Init.Mode = UART_MODE_TX_RX;          //�շ�ģʽ
    UART3_Handler.Init.OverSampling = UART_OVERSAMPLING_8;
    HAL_UART_Init(&UART3_Handler);                                       //HAL_UART_Init()��ʹ��UART1
    HAL_UART_Receive_IT(&UART3_Handler, (u8 *)&aRxBuffer, RXBUFFERSIZE); //�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������

    logUsartInit(&UART1_Handler);
    //  	__HAL_UART_CLEAR_FLAG(&UART3_Handler,UART_FLAG_RXNE);
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_Initure;

    if (huart->Instance == USART1) //����Ǵ���1�����д���1 MSP��ʼ��
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();  //ʹ��GPIOAʱ��
        __HAL_RCC_USART1_CLK_ENABLE(); //ʹ��USART1ʱ��

        GPIO_Initure.Pin = GPIO_PIN_9;            //PA9
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;      //�����������
        GPIO_Initure.Pull = GPIO_PULLUP;          //����
        GPIO_Initure.Speed = GPIO_SPEED_FAST;     //����
        GPIO_Initure.Alternate = GPIO_AF7_USART1; //����ΪUSART1
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);      //��ʼ��PA9

        GPIO_Initure.Pin = GPIO_PIN_10;      //PA10
        HAL_GPIO_Init(GPIOA, &GPIO_Initure); //��ʼ��PA10

        HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    if (huart->Instance == USART3) //����Ǵ���1�����д���1 MSP��ʼ��
    {
        __HAL_RCC_USART3_CLK_ENABLE(); //ʹ��USART1ʱ��
        __HAL_RCC_GPIOB_CLK_ENABLE();  //ʹ��GPIOAʱ��

        GPIO_Initure.Pin = GPIO_PIN_10;            //PA9
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;       //�����������
        GPIO_Initure.Pull = GPIO_NOPULL;           //����
        GPIO_Initure.Speed = GPIO_SPEED_FAST;      //����
        GPIO_Initure.Alternate = GPIO_AF7_USART3;  //����ΪUSART1
        HAL_GPIO_Init(GPIOB, &GPIO_Initure);       //��ʼ��PA9

        GPIO_Initure.Pin = GPIO_PIN_11;      			 //PA10
        HAL_GPIO_Init(GPIOB, &GPIO_Initure); 			 //��ʼ��PA10

        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE); //���������ж�
        HAL_NVIC_EnableIRQ(USART3_IRQn);           //ʹ��USART1�ж�ͨ��
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);   //��ռ���ȼ�3�������ȼ�3
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) //����Ǵ���3
    {
        if ((USART_RX_STA & 0x8000) == 0) //����δ���
        {
            if (USART_RX_STA & 0x4000) //���յ���0x0d
            {
                if (aRxBuffer[0] != 0x0a)
                    USART_RX_STA = 0; //���մ���,���¿�ʼ
                else
                {
                    USART_RX_STA |= 0x8000; //���������
                    rt_mb_send(BuleTooth_Transfer_mb, (rt_uint32_t)&USART_RX_BUF);
                }
            }
            else //��û�յ�0X0D
            {
                if (aRxBuffer[0] == 0x0d)
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = aRxBuffer[0];
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0; //�������ݴ���,���¿�ʼ����
                }
            }
        }
    }
}

//����1�жϷ������
void USART3_IRQHandler(void)
{
    uint32_t timeout = 0;
    uint32_t maxDelay = 0xFFFF;
		uint8_t t=0;
    rt_enter_critical();
    HAL_UART_IRQHandler(&UART3_Handler);                                                       //����HAL���жϴ����ú���
    while (HAL_UART_Receive_IT(&UART3_Handler, (uint8_t *)&aRxBuffer, RXBUFFERSIZE) != HAL_OK) //һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
    {
        printf("UART3_Handler.State = %x\n", UART3_Handler.State);
        UART3_Handler.State = HAL_UART_STATE_READY;
        __HAL_UNLOCK(&UART3_Handler);
        __HAL_UART_CLEAR_IDLEFLAG(&UART3_Handler);
        __HAL_UART_CLEAR_FEFLAG(&UART3_Handler);
        __HAL_UART_CLEAR_NEFLAG(&UART3_Handler);
        t = UART3_Handler.Instance->SR;
        t = UART3_Handler.Instance->DR;
        timeout++; //��ʱ����
        if (timeout > maxDelay)
            break;
    }
    if (__HAL_UART_GET_FLAG(&UART3_Handler, UART_FLAG_ORE) != RESET)
    {
        if (UART3_Handler.ErrorCode == HAL_UART_ERROR_ORE)
        {
            t = UART3_Handler.Instance->SR;
            t = UART3_Handler.Instance->DR;
            printf("����ORE���");
        }
    }
    rt_exit_critical();
    //	if((UART3_Handler.Instance->CR1 & 0x20)==0)
    //		HAL_UART_Receive_IT(&UART3_Handler,(u8 *)&aRxBuffer,RXBUFFERSIZE);

    /*	20201215������жϽ��տ�����¼��
		
		����while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY);
					��������getstate���ش���״̬���½��տ���
		
	1�����п�����HAL���bug�����������ע�͵�lock��unlock��ش��롣
	2���ڷ��ͺ��������õ�timeoutʱ�������������һ�ֽ������ˣ������ͻ�δ��������ͻ��
	3�����ڽ��������еķ���ָ�룬�����δָ��NULL���������
		�������ֿ���
*/
}
void USART1_IRQHandler(void)
{
    uint32_t timeout = 0;
    uint32_t maxDelay = 0x1FFFF;

    HAL_UART_IRQHandler(&UART1_Handler); //����HAL���жϴ����ú���
    timeout = 0;
    while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY) //�ȴ�����
    {
        timeout++; ////��ʱ����
        if (timeout > maxDelay)
            break;
    }
    timeout = 0;
    while (HAL_UART_Receive_IT(&UART1_Handler, (uint8_t *)&aRxBuffer1, 2) != HAL_OK) //һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
    {
        timeout++; //��ʱ����
        if (timeout > maxDelay)
            break;
    }
}
void Arry_Clear(uint8_t buf[], uint8_t len)
{
    for (int i = 0; i < len; i++)
    {
        buf[i] = 0;
    }
}
void Pointer_Clear(uint8_t *buf)
{
    for (int i = 0; i < 80; i++)
    {
        buf[i] = 0;
    }
    if (buf != RT_NULL)
    {
        buf = RT_NULL;
    }
}
