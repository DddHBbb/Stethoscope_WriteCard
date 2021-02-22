#include "usb_bsp.h"
#include "sys.h"  
#include "rtthread.h"
#include "usb_dcd.h"


//USB OTG �ײ�IO��ʼ��
//pdev:USB OTG�ں˽ṹ��ָ��
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
     GPIO_InitTypeDef  GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();                   //ʹ��GPIOAʱ��
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();              //ʹ��OTG FSʱ��

        //����PA11 12
    GPIO_InitStruct.Pin=GPIO_PIN_11|GPIO_PIN_12;    //PA11 12
    GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;           //����
    GPIO_InitStruct.Pull=GPIO_NOPULL;               //��������
    GPIO_InitStruct.Speed=GPIO_SPEED_HIGH;          //����
    GPIO_InitStruct.Alternate=GPIO_AF10_OTG_FS;     //����ΪOTG FS
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);         //��ʼ��
}

//USB OTG �ж�����,����USB FS�ж�
//pdev:USB OTG�ں˽ṹ��ָ��
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{  	
    HAL_NVIC_SetPriority(OTG_FS_IRQn,0,3);          //��ռ���ȼ�0�������ȼ�3
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);                //ʹ��OTG USB FS�ж� 
}

//USB OTG �ж�����,����USB FS�ж�
//pdev:USB OTG�ں˽ṹ��ָ��
void USB_OTG_BSP_DisableInterrupt(void)
{ 
		HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
}
//pdev:USB OTG�ں˽ṹ��ָ��
//state:0,�ϵ�;1,�ϵ�
void USB_OTG_BSP_DriveVBUS(USB_OTG_CORE_HANDLE *pdev, uint8_t state)
{ 
}
//pdev:USB OTG�ں˽ṹ��ָ��
void  USB_OTG_BSP_ConfigVBUS(USB_OTG_CORE_HANDLE *pdev)
{ 
} 
//USB_OTG us����ʱ����
//usec:Ҫ��ʱ��us��.
void USB_OTG_BSP_uDelay (const uint32_t usec)
{ 
  // 	delay_us(usec);
}
//USB_OTG ms����ʱ����
//msec:Ҫ��ʱ��ms��.
void USB_OTG_BSP_mDelay (const uint32_t msec)
{  
	//delay_ms(msec);
}
// �Ƴ�U���豸
void usbd_CloseMassStorage(USB_OTG_CORE_HANDLE *pdev)
{
	DCD_DevDisconnect(pdev);

	USB_OTG_BSP_DisableInterrupt();	/* �ر�USB��ص������ж� */
}

















