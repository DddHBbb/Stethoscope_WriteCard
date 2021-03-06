#include "board.h"
#include "rtthread.h"
#include "usbd_msc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include "usbd_msc_bot.h"
#include "usb_bsp.h"
#include "demo.h"
#include "platform.h"
#include "st25r95_com.h"

/*开启任务宏定义*/
#define Creat_Wav_Player
#define Creat_USB_Transfer
#define Creat_NFC_Transfer
#define Creat_Dispose
#define Creat_Write_Card

/***********************函数声明区*******************************/
static void Wav_Player_Task  (void *parameter);
static void USB_Transfer_Task(void *parameter);
static void NFC_Transfer_Task(void *parameter);
static void Dispose_Task     (void *parameter);
/***********************声明返回区*******************************/
USB_OTG_CORE_HANDLE USB_OTG_dev;
extern vu8 USB_STATUS_REG; //USB状态
extern vu8 bDeviceState;   //USB连接 情况
extern rt_timer_t LowPWR_timer;
extern void Adjust_Volume(void);
/***********************全局变量区*******************************/
static char *NFCTag_UID_RECV 						= NULL;
static char *NFCTag_CustomID_RECV			  = NULL;
static char *Rev_From_BT 								= NULL;
static char *The_Auido_Name 						= NULL;
char Last_Audio_Name[50] 								= "noway";
char dataOut[COM_XFER_SIZE] 						= {0};
uint8_t TT2Tag[NFCT2_MAX_TAGMEMORY] 		= {0};
const char ARM[][9]										  = {"24270042","24270043"};

//任务句柄
static rt_thread_t Wav_Player 					= RT_NULL;
static rt_thread_t USB_Transfer 				= RT_NULL;
static rt_thread_t NFC_Transfer 				= RT_NULL;
static rt_thread_t Dispose 							= RT_NULL;

//信号量句柄
rt_mutex_t USBorAudioUsingSDIO_Mutex 		= RT_NULL;

//邮箱句柄
rt_mailbox_t NFC_TagID_mb = RT_NULL;
rt_mailbox_t BuleTooth_Transfer_mb 			= RT_NULL;
rt_mailbox_t NFC_SendMAC_mb 						= RT_NULL;
rt_mailbox_t The_Auido_Name_mb 					= RT_NULL;
rt_mailbox_t NFCTag_CustomID_mb 				= RT_NULL;
rt_mailbox_t LOW_PWR_mb 								= RT_NULL;
rt_mailbox_t Start_Playing_mb 					= RT_NULL;
rt_mailbox_t Stop_Playing_mb 						= RT_NULL;
rt_mailbox_t Write_Card_mb 						  = RT_NULL;

//事件句柄
rt_event_t AbortWavplay_Event 					= RT_NULL;
rt_event_t PlayWavplay_Event 						= RT_NULL;
rt_event_t OLED_Display_Event 					= RT_NULL;
/****************************************************************/

/****************************************
  * @brief  WAV音频播放函数
  * @param  无
  * @retval 无
  ***************************************/
void Wav_Player_Task(void *parameter)
{
    static uint8_t DataToBT[50] = {0};
    printf("Wav_Player_Task\n");

    while (1)
    {
			if ((rt_mb_recv(NFCTag_CustomID_mb, (rt_uint32_t *)&NFCTag_CustomID_RECV, RT_WAITING_NO)) == RT_EOK)
			{
				rt_sprintf((char *)DataToBT, "%x%x%02x%02x\r\n", NFCTag_CustomID_RECV[0], NFCTag_CustomID_RECV[1],
																												 NFCTag_CustomID_RECV[2], NFCTag_CustomID_RECV[3]); //11
				rt_kprintf("DataToBT=%s\n", DataToBT);
				Arry_Clear((uint8_t *)DataToBT, sizeof(DataToBT));
			}
        rt_thread_delay(500); //1000ms        
    }
}
/****************************************
  * @brief  USB大容量存储函数
  * @param  无
  * @retval 无
  ***************************************/
void USB_Transfer_Task(void *parameter)
{
    printf("USB_Transfer_Task\n");
    while (1)
    {
			if (HAL_GPIO_ReadPin(GPIOC, USB_Connect_Check_PIN) == GPIO_PIN_RESET)
        {
					SD_Init();
					MSC_BOT_Data = (uint8_t *)rt_malloc(MSC_MEDIA_PACKET); //申请内存
					USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_MSC_cb, &USR_cb);
					while(1)
					{
						ChargeDisplay();
						rt_thread_delay(1000); //1000ms
						if (HAL_GPIO_ReadPin(GPIOC, USB_Connect_Check_PIN)  != GPIO_PIN_RESET)
						{
							usbd_CloseMassStorage(&USB_OTG_dev);
							rt_free(MSC_BOT_Data);	
							break;
						}
					}				
				}				
				rt_thread_delay(1000); //1000ms
    }     
}

void Dispose_Task(void *parameter)
{
    while (1)
    {
        if (HAL_GPIO_ReadPin(GPIOC, USB_Connect_Check_PIN) == GPIO_PIN_SET)
        {
            BattChek();
            OLED_Refresh_Gram();
        }
        rt_thread_delay(500); 
    }
}
/****************************************
  * @brief  NFC连接处理函数
  * @param  无
  * @retval 无
  ***************************************/
void NFC_Transfer_Task(void *parameter)
{
    printf("NFC_Transfer_Task\n");
		OLED_Clear();
    Show_String(0, 0, (uint8_t *)"听诊器写卡工装");
		Show_String(0, 16, (uint8_t *)"写入范围 1~43");
		Show_String(0, 32, (uint8_t *)"当前输入:");
	  OLED_Refresh_Gram();
	
    if (!demoIni()) //初始化cr95hf
        platformLog("Initialization failed..\r\n");
    ConfigManager_HWInit();                                          //初始化读卡
    while (1)
    {
				Adjust_Volume();
        demoCycle();
        rt_thread_delay(1); //5ms
    }
}
/****************************************
  * @brief  任务创建函数，所有任务均在此处创建 
  * @param  无
  * @retval 无
  ***************************************/
void Task_init(void)
{
#ifdef Creat_Wav_Player
    /*音乐播放器任务*/
    Wav_Player = rt_thread_create	 ("Wav_Player_Task", 	 Wav_Player_Task, 	RT_NULL, 512, 1, 100);
    if (Wav_Player != RT_NULL)
        rt_thread_startup(Wav_Player);
#endif
#ifdef Creat_USB_Transfer
    /*USB大容量存储任务*/
    USB_Transfer = rt_thread_create("USB_Transfer_Task", USB_Transfer_Task, RT_NULL, 512,  0, 20);
    if (USB_Transfer != RT_NULL)
        rt_thread_startup(USB_Transfer);
#endif
#ifdef Creat_NFC_Transfer
    /*NFC任务*/
    NFC_Transfer = rt_thread_create("NFC_Transfer_Task", NFC_Transfer_Task, RT_NULL, 1024, 1, 20);
    if (NFC_Transfer != RT_NULL)
        rt_thread_startup(NFC_Transfer);
#endif
#ifdef Creat_Dispose
    Dispose = rt_thread_create		 ("Status_Reflash", 	 Dispose_Task, 			RT_NULL, 1024, 2, 20);
    if (Dispose != RT_NULL)
        rt_thread_startup(Dispose);
#endif
}
/****************************************
  * @brief  信号量创建函数，二值，互斥 
  * @param  无
  * @retval 无
  ***************************************/
void Semaphore_init(void)
{
    USBorAudioUsingSDIO_Mutex = rt_mutex_create("USBorAudioUsingSDIO_Mutex", RT_IPC_FLAG_PRIO);
}
/****************************************
	* @brief  邮箱创建函数  
  * @param  无
  * @retval 无
  ***************************************/
void Mailbox_init(void)
{
    NFCTag_CustomID_mb 		= rt_mb_create("NFCTag_CustomID_mb", 		1, RT_IPC_FLAG_FIFO);
    NFC_TagID_mb 			 		= rt_mb_create("NFC_TagID_mb", 					1, RT_IPC_FLAG_FIFO);
    BuleTooth_Transfer_mb = rt_mb_create("BuleTooth_Transfer_mb", 1, RT_IPC_FLAG_FIFO);
    NFC_SendMAC_mb 				= rt_mb_create("NFC_SendMAC_mb", 				1, RT_IPC_FLAG_FIFO);
    The_Auido_Name_mb 		= rt_mb_create("The_Auido_Name_mb", 		1, RT_IPC_FLAG_FIFO);
    LOW_PWR_mb 						= rt_mb_create("LOW_PWR_mb", 						1, RT_IPC_FLAG_FIFO);
		Start_Playing_mb 			= rt_mb_create("Start_Playing_mb", 			1, RT_IPC_FLAG_FIFO);
	  Stop_Playing_mb 			= rt_mb_create("Stop_Playing_mb", 			1, RT_IPC_FLAG_FIFO);
		Write_Card_mb         = rt_mb_create("Write_Card_mb", 			  1, RT_IPC_FLAG_FIFO);
}
/****************************************
  * @brief  事件创建函数 
  * @param  无
  * @retval 无
  ***************************************/
void Event_init(void)
{
    AbortWavplay_Event = rt_event_create("AbortWavplay_Event",  RT_IPC_FLAG_FIFO);
    PlayWavplay_Event  = rt_event_create("PlaytWavplay_Event",  RT_IPC_FLAG_FIFO);
    OLED_Display_Event = rt_event_create("OLED_Display_Event",  RT_IPC_FLAG_FIFO);
}













