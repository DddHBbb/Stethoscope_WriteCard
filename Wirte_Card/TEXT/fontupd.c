#include "fontupd.h"
#include "ff.h"
#include "w25qxx.h"
#include "string.h"
#include "usart.h"
#include "rtthread.h"

//�ֿ�����ռ�õ�����������С(4���ֿ�+unigbk��+�ֿ���Ϣ=6302984�ֽ�,Լռ1539��W25QXX����,һ������4K�ֽ�)
#define FONTSECSIZE 1539
//�ֿ�����ʼ��ַ
#define FONTINFOADDR 1024 * 1024 * 1 //Apollo STM32�������Ǵ�25M��ַ�Ժ�ʼ����ֿ�                      \
                                     //ǰ��25M��fatfsռ����.                                                       \
                                     //25M�Ժ����4���ֿ�+UNIGBK.BIN,�ܴ�С6.01M,���ֿ�ռ����,���ܶ�! \
                                     //31.01M�Ժ�,�û���������ʹ��.

//���������ֿ������Ϣ����ַ����С��
_font_info ftinfo;

//�ֿ����ڴ����е�·��
u8 *const GBK_PATH[5] =
    {
        "/SYSTEM/FONT/UNIGBK.BIN", //UNIGBK.BIN�Ĵ��λ��
        "/SYSTEM/FONT/GBK12.FON",  //GBK12�Ĵ��λ��
        "/SYSTEM/FONT/GBK16.FON",  //GBK16�Ĵ��λ��
        "/SYSTEM/FONT/GBK24.FON",  //GBK24�Ĵ��λ��
        "/SYSTEM/FONT/GBK32.FON",  //GBK32�Ĵ��λ��
};
//����ʱ����ʾ��Ϣ
u8 *const UPDATE_REMIND_TBL[5] =
    {
        "Updating UNIGBK.BIN", //��ʾ���ڸ���UNIGBK.bin
        "Updating GBK12.FON ", //��ʾ���ڸ���GBK12
        "Updating GBK16.FON ", //��ʾ���ڸ���GBK16
        "Updating GBK24.FON ", //��ʾ���ڸ���GBK24
        "Updating GBK32.FON ", //��ʾ���ڸ���GBK32
};
//����ĳһ��
//x,y:����
//size:�����С
//fxpath:·��
//fx:���µ����� 0,ungbk;1,gbk12;2,gbk16;3,gbk24;4,gbk32;
//����ֵ:0,�ɹ�;����,ʧ��.
u8 updata_fontx(u8 *fxpath, u8 fx)
{
    u32 flashaddr = 0;
    FIL *fftemp;
    u8 *tempbuf;
    u8 res;
    u16 bread;
    u32 offx = 0;
    u8 rval = 0;
    fftemp = (FIL *)rt_malloc(sizeof(FIL)); //�����ڴ�
    if (fftemp == NULL)
        rval = 1;
    tempbuf = rt_malloc(4096); //����4096���ֽڿռ�
    if (tempbuf == NULL)
        rval = 1;
    res = f_open(fftemp, (const TCHAR *)fxpath, FA_READ);
    if (res)
        rval = 2; //���ļ�ʧ��
    if (rval == 0)
    {
        switch (fx)
        {
        case 0:                                              //����UNIGBK.BIN
            ftinfo.ugbkaddr = FONTINFOADDR + sizeof(ftinfo); //��Ϣͷ֮�󣬽���UNIGBKת�����
            ftinfo.ugbksize = fftemp->obj.objsize;           //UNIGBK��С
            flashaddr = ftinfo.ugbkaddr;
            break;
        case 1:
            ftinfo.f12addr = ftinfo.ugbkaddr + ftinfo.ugbksize; //UNIGBK֮�󣬽���GBK12�ֿ�
            ftinfo.gbk12size = fftemp->obj.objsize;             //GBK12�ֿ��С
            flashaddr = ftinfo.f12addr;                         //GBK12����ʼ��ַ
            break;
        case 2:
            ftinfo.f16addr = ftinfo.f12addr + ftinfo.gbk12size; //GBK12֮�󣬽���GBK16�ֿ�
            ftinfo.gbk16size = fftemp->obj.objsize;             //GBK16�ֿ��С
            flashaddr = ftinfo.f16addr;                         //GBK16����ʼ��ַ
            break;
        case 3:
            ftinfo.f24addr = ftinfo.f16addr + ftinfo.gbk16size; //GBK16֮�󣬽���GBK24�ֿ�
            ftinfo.gbk24size = fftemp->obj.objsize;             //GBK24�ֿ��С
            flashaddr = ftinfo.f24addr;                         //GBK24����ʼ��ַ
            break;
        case 4:
            ftinfo.f32addr = ftinfo.f24addr + ftinfo.gbk24size; //GBK24֮�󣬽���GBK32�ֿ�
            ftinfo.gbk32size = fftemp->obj.objsize;             //GBK32�ֿ��С
            flashaddr = ftinfo.f32addr;                         //GBK32����ʼ��ַ
            break;
        }
        while (res == FR_OK) //��ѭ��ִ��
        {
            res = f_read(fftemp, tempbuf, 4096, (UINT *)&bread); //��ȡ����
            if (res != FR_OK)
                break;                                     //ִ�д���
            W25QXX_Write(tempbuf, offx + flashaddr, 4096); //��0��ʼд��4096������
            offx += bread;
            if (bread != 4096)
                break; //������.
        }
        f_close(fftemp);
    }
    rt_free(fftemp);  //�ͷ��ڴ�
    rt_free(tempbuf); //�ͷ��ڴ�
    return res;
}
//���������ļ�,UNIGBK,GBK12,GBK16,GBK24,GBK32һ�����
//x,y:��ʾ��Ϣ����ʾ��ַ
//size:�����С
//src:�ֿ���Դ����."0:",SD��;"1:",FLASH��,"2:"NAND��,"3:",U��.
//��ʾ��Ϣ�����С
//����ֵ:0,���³ɹ�;
//		 ����,�������.
u8 update_font(u8 *src)
{
    u8 *pname;
    u32 *buf;
    u8 res = 0;
    u16 i, j;
    FIL *fftemp;
    u8 rval = 0;
    res = 0XFF;
    ftinfo.fontok = 0XFF;
    pname = rt_malloc(100);                 //����100�ֽ��ڴ�
    buf = rt_malloc(4096);                  //����4K�ֽ��ڴ�
    fftemp = (FIL *)rt_malloc(sizeof(FIL)); //�����ڴ�
    if (buf == NULL || pname == NULL || fftemp == NULL)
    {
        rt_free(fftemp);
        rt_free(pname);
        rt_free(buf);
        return 5; //�ڴ�����ʧ��
    }
    for (i = 0; i < 5; i++) //�Ȳ����ļ�UNIGBK,GBK12,GBK16,GBK24,GBK32�Ƿ�����
    {
        strcpy((char *)pname, (char *)src);         //copy src���ݵ�pname
        strcat((char *)pname, (char *)GBK_PATH[i]); //׷�Ӿ����ļ�·��
        printf("open_file");
        res = f_open(fftemp, (const TCHAR *)pname, FA_READ); //���Դ�
        if (res)
        {
            printf("open_fail");
            rval |= 1 << 7; //��Ǵ��ļ�ʧ��
            break;          //������,ֱ���˳�
        }
    }
    rt_free(fftemp); //�ͷ��ڴ�
    if (rval == 0)   //�ֿ��ļ�������.
    {
        for (i = 0; i < FONTSECSIZE; i++) //�Ȳ����ֿ�����,���д���ٶ�
        {
            W25QXX_Read((u8 *)buf, ((FONTINFOADDR / 4096) + i) * 4096, 4096); //������������������
            for (j = 0; j < 1024; j++)                                        //У������
            {
                if (buf[j] != 0XFFFFFFFF)
                    break; //��Ҫ����
            }
            if (j != 1024)
                W25QXX_Erase_Sector((FONTINFOADDR / 4096) + i); //��Ҫ����������
        }
        for (i = 0; i < 5; i++) //���θ���UNIGBK,GBK12,GBK16,GBK24,GBK32
        {
            strcpy((char *)pname, (char *)src);         //copy src���ݵ�pname
            strcat((char *)pname, (char *)GBK_PATH[i]); //׷�Ӿ����ļ�·��
            printf("update fontlib");
            res = updata_fontx(pname, i); //�����ֿ�
            if (res)
            {
                rt_free(buf);
                rt_free(pname);
                return 1 + i;
            }
        }
        printf("all_ok");
        //ȫ�����º���
        ftinfo.fontok = 0XAA;
        W25QXX_Write((u8 *)&ftinfo, FONTINFOADDR, sizeof(ftinfo)); //�����ֿ���Ϣ
    }
    rt_free(pname); //�ͷ��ڴ�
    rt_free(buf);
    return rval; //�޴���.
}
//��ʼ������
//����ֵ:0,�ֿ����.
//		 ����,�ֿⶪʧ
u8 font_init(void)
{
    u8 t = 0;
    W25QXX_Init();
    while (t < 10) //������ȡ10��,���Ǵ���,˵��ȷʵ��������,�ø����ֿ���
    {
        t++;
        W25QXX_Read((u8 *)&ftinfo, FONTINFOADDR, sizeof(ftinfo)); //����ftinfo�ṹ������
        if (ftinfo.fontok == 0XAA)
            break;
        rt_thread_delay(10);
    }
    if (ftinfo.fontok != 0XAA)
        return 1;
    return 0;
}