#include "oled.h"
#include "RTE_Components.h" // Component selection
#include "stdlib.h"
#include "oledfont.h"
#include "rtthread.h"
#include "text.h"
#include "string.h"
#include "GPIOConfig.h"
#include "D_delay.h"
#include "usart.h"

void Show_chars(uint8_t x, uint8_t y, uint8_t xend, uint8_t yend, uint8_t chr);
const uint8_t gImage_12[504];
//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127
u8 OLED_GRAM[128][8];
uint8_t BodyData[16];

//�����Դ浽LCD
void OLED_Refresh_Gram(void)
{
    rt_enter_critical();
    u8 i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); //����ҳ��ַ��0~7��
        OLED_WR_Byte(0x00, OLED_CMD);     //������ʾλ�á��е͵�ַ
        OLED_WR_Byte(0x10, OLED_CMD);     //������ʾλ�á��иߵ�ַ
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
    rt_exit_critical();
}
//��SSD1306д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat, u8 cmd)
{
    u8 i;
    OLED_RS = cmd; //д����
    OLED_CS = 0;
    for (i = 0; i < 8; i++)
    {
        OLED_SCLK = 0;
        if (dat & 0x80)
            OLED_SDIN = 1;
        else
            OLED_SDIN = 0;
        OLED_SCLK = 1;
        dat <<= 1;
    }
    OLED_CS = 1;
    OLED_RS = 1;
}

//����OLED��ʾ
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC����
    OLED_WR_Byte(0X14, OLED_CMD); //DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD); //DISPLAY ON
}
//�ر�OLED��ʾ
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC����
    OLED_WR_Byte(0X10, OLED_CMD); //DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD); //DISPLAY OFF
}
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!
void OLED_Clear(void)
{
    u8 i, n;
    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0X00;
    OLED_Refresh_Gram(); //������ʾ
}
//����
//x:0~127
//y:0~63
//t:1 ��� 0,���
void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
    u8 pos, bx, temp = 0;
    if (x > 127 || y > 63)
        return; //������Χ��.
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}
//x1,y1,x2,y2 �������ĶԽ�����
//ȷ��x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63
//dot:0,���;1,���
void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot)
{
    u8 x, y;
    for (x = x1; x <= x2; x++)
    {
        for (y = y1; y <= y2; y++)
            OLED_DrawPoint(x, y, dot);
    }
}
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ
//size:ѡ������ 12/16/24
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode)
{
    u8 temp, t, t1;
    u8 y0 = y;
    u8 csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2); //�õ�����һ���ַ���Ӧ������ռ���ֽ���
    chr = chr - ' ';                                           //�õ�ƫ�ƺ��ֵ
    for (t = 0; t < csize; t++)
    {
        if (size == 12)
            temp = asc2_1206[chr][t]; //����1206����
        else if (size == 16)
            temp = asc2_1608[chr][t]; //����1608����
        else if (size == 24)
            temp = asc2_2412[chr][t]; //����2412����
        else
            return; //û�е��ֿ�
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}
//m^n����
u32 mypow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--)
        result *= m;
    return result;
}
//��ʾ2������
//x,y :�������
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size, u8 mode)
{
    u8 t, temp;
    u8 enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size, mode);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, mode);
    }
}
//��ʾ�ַ���
//x,y:�������
//size:�����С
//*p:�ַ�����ʼ��ַ
void OLED_ShowString(u8 x, u8 y, const u8 *p, u8 size)
{
    while ((*p <= '~') && (*p >= ' ')) //�ж��ǲ��ǷǷ��ַ�!
    {
        if (x > (128 - (size / 2)))
        {
            x = 0;
            y += size;
        }
        if (y > (64 - size))
        {
            y = x = 0;
            OLED_Clear();
        }
        OLED_ShowChar(x, y, *p, size, 1);
        x += size / 2;
        p++;
    }
}
//��ʼ��SSD1306
void OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOA_CLK_ENABLE(); //ʹ��GPIOAʱ��
    __HAL_RCC_GPIOD_CLK_ENABLE(); //ʹ��GPIODʱ��

    //GPIO��ʼ������
    GPIO_Initure.Pin = GPIO_PIN_15;          //PA15
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; //�������
    GPIO_Initure.Pull = GPIO_PULLUP;         //����
    GPIO_Initure.Speed = GPIO_SPEED_FAST;    //����
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);     //��ʼ��

    //PB4,7
    GPIO_Initure.Pin = GPIO_PIN_4 | GPIO_PIN_0;
    HAL_GPIO_Init(GPIOD, &GPIO_Initure); //��ʼ��

    //PC6,7
    GPIO_Initure.Pin = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOD, &GPIO_Initure); //��ʼ��

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

    OLED_SDIN = 1;
    OLED_SCLK = 1;

    OLED_CS = 1;
    OLED_RS = 1;

    OLED_RST = 0;
    delay_ms(100);
    OLED_RST = 1;

    OLED_WR_Byte(0xAE, OLED_CMD); //�ر���ʾ
    OLED_WR_Byte(0xD5, OLED_CMD); //����ʱ�ӷ�Ƶ����,��Ƶ��
    OLED_WR_Byte(80, OLED_CMD);   //[3:0],��Ƶ����;[7:4],��Ƶ��
    OLED_WR_Byte(0xA8, OLED_CMD); //��������·��
    OLED_WR_Byte(0X3F, OLED_CMD); //Ĭ��0X3F(1/64)
    OLED_WR_Byte(0xD3, OLED_CMD); //������ʾƫ��
    OLED_WR_Byte(0X00, OLED_CMD); //Ĭ��Ϊ0
    OLED_WR_Byte(0x40, OLED_CMD); //������ʾ��ʼ�� [5:0],����.
    OLED_WR_Byte(0x8D, OLED_CMD); //��ɱ�����
    OLED_WR_Byte(0x14, OLED_CMD); //bit2������/�ر�
    OLED_WR_Byte(0x20, OLED_CMD); //�����ڴ��ַģʽ
    OLED_WR_Byte(0x02, OLED_CMD); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
    OLED_WR_Byte(0xA1, OLED_CMD); //���ض�������,bit0:0,0->0;1,0->127;
    OLED_WR_Byte(0xC0, OLED_CMD); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
    OLED_WR_Byte(0xDA, OLED_CMD); //����COMӲ����������
    OLED_WR_Byte(0x12, OLED_CMD); //[5:4]����
    OLED_WR_Byte(0x81, OLED_CMD); //�Աȶ�����
    OLED_WR_Byte(0xEF, OLED_CMD); //1~255;Ĭ��0X7F (��������,Խ��Խ��)
    OLED_WR_Byte(0xD9, OLED_CMD); //����Ԥ�������
    OLED_WR_Byte(0xf1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
    OLED_WR_Byte(0xDB, OLED_CMD); //����VCOMH ��ѹ����
    OLED_WR_Byte(0x30, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
    OLED_WR_Byte(0xA4, OLED_CMD); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
    OLED_WR_Byte(0xA6, OLED_CMD); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ
    OLED_WR_Byte(0xAF, OLED_CMD); //������ʾ
                                  //OLED_Clear();
}

//��ָ����ַ��ʼ��ʾһ��ASCII�ַ�
//x,y:��ʾ��ʼ����.
//xend,yend:x,y ������յ�����
//chr:�ַ�
void Show_chars(uint8_t x, uint8_t y, uint8_t xend, uint8_t yend, uint8_t chr)
{
    uint8_t temp;
    uint8_t pos, t;
    uint16_t y0 = y;
    uint8_t color = 1;
    uint8_t size = 12;

    if (chr > ' ')
        chr = chr - ' '; //�õ�ƫ�ƺ��ֵ
    else
        chr = 0; //С�ڿո��һ���ÿո����,����TAB��(��ֵΪ9)
    for (pos = 0; pos < size; pos++)
    {
        if (size == 12)
            temp = asc2_1206[chr][pos]; //����1206����
        else
            temp = asc2_1608[chr][pos]; //����1608����
        if (x > xend)
            return; //��������
        for (t = 0; t < 8; t++)
        {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, color);
            else
                OLED_DrawPoint(x, y, !color);
            temp <<= 1;
            y++;
            if ((y - y0) == size || (y > yend))
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

//��ָ��λ�ÿ�ʼ,��ʾָ�����ȷ�Χ���ַ���.
//x,y:��ʾ��ʼ����.
//xend:x�������ֹ����
//str:�ַ���
void Show_String(uint8_t x, uint8_t y, uint8_t *str)
{
    u8 bHz = 0; //�ַ���������
    uint8_t size = 12;
    uint8_t xend = 0, yend = 0;

    xend = (strlen((const char *)str) * 14);
    yend = y + 12;
    while (*str != 0) //����δ����
    {
        if (!bHz)
        {
            if (*str > 0x80)
                bHz = 1; //����
            else         //�ַ�
            {
                Show_chars(x, y, xend, yend, *str); //��ʾһ���ַ�
                if ((xend - x) > size / 2)
                    x += size / 2; //�ַ�,Ϊȫ�ֵ�һ��
                else
                    x += xend - x; //δ��ȫ��ʾ
                if (x >= xend)
                    return; //������,�˳�
                str++;
            }
        }
        else //����
        {
            bHz = 0;                                //�к��ֿ�
            Show_Font(x, y, xend, yend, size, str); //��ʾ�������,������ʾ
            if ((xend - x) > size)
                x += size; //�ַ�,Ϊȫ�ֵ�һ��
            else
                x += xend - x; //δ��ȫ��ʾ
            if (x >= xend)
                return; //������,�˳�
            str += 2;
        }
    }
}
//����
//x1,y1:�������
//x2,y2:�յ�����
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; //������������
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        incx = 1; //���õ�������
    else if (delta_x == 0)
        incx = 0; //��ֱ��
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //ˮƽ��
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; //ѡȡ��������������
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++) //�������
    {
        OLED_DrawPoint(uRow, uCol, dot); //����
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}
//������
//(x1,y1),(x2,y2):���εĶԽ�����
void OLED_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    OLED_DrawLine(x1, y1, x2, y1, 1);
    OLED_DrawLine(x1, y1, x1, y2, 1);
    OLED_DrawLine(x1, y2, x2, y2, 1);
    OLED_DrawLine(x2, y1, x2, y2, 1);
}
void BattChek(void)
{
    uint8_t xpos = 106, ypos = 2;

    if (!HAL_GPIO_ReadPin(GPIOD, Batt_25))
    {
        if (!HAL_GPIO_ReadPin(GPIOB, Batt_50))
        {
            if (!HAL_GPIO_ReadPin(GPIOB, Batt_75))
            {
                if (!HAL_GPIO_ReadPin(GPIOB, Batt_100))
                {
                    //����100%
                    OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 1);
                }
                else
                {
                    //����75%
                    OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
                }
                OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 1);
            }
            else
            {
                //����50%
                OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 0);
                OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
            }
            OLED_Fill(xpos + 6, ypos, xpos + 8, ypos + 5, 1);
        }
        else
        {
            //����25%
            OLED_Fill(xpos + 6, ypos, xpos + 8, ypos + 5, 0);
            OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 0);
            OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
        }
        OLED_Fill(xpos + 2, ypos, xpos + 4, ypos + 5, 1);
    }
    else
    {
        //����0%��
        OLED_Fill(xpos + 2, ypos, xpos + 4, ypos + 5, 0);
        OLED_Fill(xpos + 6, ypos, xpos + 8, ypos + 5, 0);
        OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
        OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 0);
    }
    OLED_Fill(xpos + 18, ypos + 1, xpos + 20, ypos + 4, 1);
    OLED_DrawRectangle(xpos, ypos - 2, xpos + 18, ypos + 7);
}
void ChargeDisplay(void)
{
    uint8_t xpos = 106, ypos = 2;
    OLED_Fill(xpos + 18, ypos + 1, xpos + 20, ypos + 4, 1);
    OLED_DrawRectangle(xpos, ypos - 2, xpos + 18, ypos + 7);

    if (HAL_GPIO_ReadPin(GPIOD, Batt_25))
    {
        OLED_Fill(xpos + 2, ypos, xpos + 4, ypos + 5, 1);
        OLED_Fill(xpos + 6, ypos, xpos + 8, ypos + 5, 0);
        OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 0);
        OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
    }
    else
    {
        OLED_Fill(xpos + 2, ypos, xpos + 4, ypos + 5, 1);
    }
    OLED_Refresh_Gram();
    delay_ms(500);

    if (HAL_GPIO_ReadPin(GPIOB, Batt_50))
    {
        OLED_Fill(xpos + 6, ypos, xpos + 8, ypos + 5, 1);
        OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 0);
        OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
    }
    else
    {
        OLED_Fill(xpos + 6, ypos, xpos + 8, ypos + 5, 1);
    }
    OLED_Refresh_Gram();
    delay_ms(500);

    if (HAL_GPIO_ReadPin(GPIOB, Batt_75))
    {
        OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 1);
        OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
    }
    else
    {
        OLED_Fill(xpos + 10, ypos, xpos + 12, ypos + 5, 1);
    }
    OLED_Refresh_Gram();
    delay_ms(500);

    if (!(HAL_GPIO_ReadPin(GPIOB, Batt_75)) && (HAL_GPIO_ReadPin(GPIOB, Batt_100))) //����75��δ��100
    {
        OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 1);
        OLED_Refresh_Gram();
        delay_ms(500);
        OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 0);
        OLED_Refresh_Gram();
        delay_ms(500);
    }
    else if (HAL_GPIO_ReadPin(GPIOB, Batt_100) || (!HAL_GPIO_ReadPin(GPIOB, Batt_100))) //δ�����������ʾ�ã��������̶���ʾ�� 
    {
        OLED_Fill(xpos + 14, ypos, xpos + 16, ypos + 5, 1);
        OLED_Refresh_Gram();
        delay_ms(500);
    }
}
//�������������嶯��ÿ֡�ĳ���
#define MOVIE_XSIZE_1 64
#define MOVIE_YSIZE_1 63
#define FRAME_SIZE_1 MOVIE_XSIZE_1 *MOVIE_YSIZE_1 / 8

void Movie_Show_Img(uint8_t x, uint8_t y, uint32_t picx)
{
    uint16_t temp, t, t1;
    uint16_t y0 = y;
    picx *= FRAME_SIZE_1;
    for (t = 0; t < FRAME_SIZE_1; t++)
    {
        temp = gImage_12[picx + t]; //�õ�ͼƬ��һ���ֽ�����
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, 1);
            else
                OLED_DrawPoint(x, y, 0);
            temp <<= 1;
            y++;
            if ((y - y0) == MOVIE_YSIZE_1)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
    OLED_Refresh_Gram(); //������ʾ��OLED
}
/*��̬��ʾ��ǰλ��*/
void DispCurrentPosition(const uint8_t *pbuf)
{
    const uint8_t *buf = pbuf;
    uint16_t len = strlen((const char *)buf) * 6; /*12������*/
    static uint8_t old_data;
    if (old_data != BodyData[0]) /*�Ƿ���ϴ���ͬһ��λ��*/
    {
        old_data = BodyData[0];
    }
    if (len > 127) /*��ʾ���ݳ�����������*/
    {
        Show_String(48, 36, (uint8_t *)buf);
    }
    else /*��ʾδ������������*/
    {
        Show_String(48, 36, (uint8_t *)buf);
    }
}
void BluetoothDisp(u8 t)
{
    uint8_t xpos = 95, ypos = 0;
    OLED_DrawLine(xpos, ypos + 3, xpos + 6, ypos + 9, t);
    OLED_DrawLine(xpos, ypos + 9, xpos + 6, ypos + 3, t);
    OLED_DrawLine(xpos + 3, ypos, xpos + 3, ypos + 12, t);
    OLED_DrawLine(xpos + 3, ypos, xpos + 6, ypos + 3, t);
    OLED_DrawLine(xpos + 3, ypos + 12, xpos + 6, ypos + 9, t);
    OLED_DrawLine(xpos, ypos + 2, xpos + 4, ypos + 6, t);
    OLED_Refresh_Gram();
}
extern UART_HandleTypeDef UART3_Handler;

void Battery_Capacity_Transmit(void)
{
    uint8_t Capacity = 0;
    static uint8_t aCapacity[20] = {0};

    if (HAL_GPIO_ReadPin(GPIOB, Batt_100) == GPIO_PIN_RESET)
        Capacity = 100;
    else if (HAL_GPIO_ReadPin(GPIOB, Batt_75) == GPIO_PIN_RESET)
        Capacity = 75;
    else if (HAL_GPIO_ReadPin(GPIOB, Batt_50) == GPIO_PIN_RESET)
        Capacity = 50;
    else if (HAL_GPIO_ReadPin(GPIOD, Batt_25) == GPIO_PIN_RESET)
        Capacity = 25;
    else
        Capacity = 0;

    rt_sprintf((char *)aCapacity, "BatteryLevel:%d\r\n", Capacity); //13
    rt_enter_critical();
    HAL_UART_Transmit(&UART3_Handler, (uint8_t *)aCapacity, strlen((const char *)(aCapacity)), 1000);
    while (__HAL_UART_GET_FLAG(&UART3_Handler, UART_FLAG_TC) != SET)
        ; //�ȴ����ͽ���
    rt_exit_critical();
}
void VolumeShow(uint8_t x, uint8_t y, uint8_t xsize, uint8_t ysize, uint32_t picx, const uint8_t str[])
{
    uint16_t temp, t, t1;
    uint16_t y0 = y;
    uint16_t frame_size = xsize * ysize / 8;
    picx *= frame_size;
    for (t = 0; t < frame_size; t++)
    {
        temp = str[picx + t];
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, 1);
            else
                OLED_DrawPoint(x, y, 0);
            temp <<= 1;
            y++;
            if ((y - y0) == ysize)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}


const uint8_t gImage_12[504] = { /* 0X01,0X01,0X3F,0X00,0X40,0X00, */
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X3F,0XC0,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0XFE,0X00,0X00,0X00,
0X00,0X00,0X3F,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XE0,0X00,0X00,
0X00,0X03,0XFF,0XFF,0XFF,0XF8,0X00,0X00,0X00,0X07,0XFF,0XFF,0XFF,0XFC,0X00,0X00,
0X00,0X0F,0XFF,0XFF,0XFF,0XF8,0X04,0X00,0X00,0X3F,0XFF,0XFF,0XFF,0XF8,0X04,0X00,
0X00,0X7F,0XFF,0XFF,0XFF,0XF0,0X07,0XFC,0X00,0XFF,0XFF,0XFF,0XFF,0XE0,0X07,0XFC,
0X01,0XFF,0XFF,0XFF,0XFF,0XE0,0X04,0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XC0,0X04,0X00,
0X03,0XFF,0XFF,0XFF,0XFF,0XC0,0X00,0X00,0X07,0XFF,0XFF,0XFF,0XFF,0X80,0X07,0XFC,
0X07,0XFF,0XFF,0XC1,0XFF,0X80,0X06,0XCC,0X0F,0XFF,0XFE,0X00,0X7F,0X80,0X04,0XCC,
0X0F,0XFF,0XF8,0XFA,0X3F,0X00,0X04,0XCC,0X1F,0XFF,0XF1,0XFE,0X0F,0X00,0X04,0XCC,
0X1F,0XFF,0XE7,0XD6,0X07,0X00,0X00,0X00,0X3F,0XFF,0XCF,0XD4,0X22,0X00,0X00,0X00,
0X3F,0XFF,0XCE,0XB4,0X32,0X00,0X07,0XFC,0X3F,0XFF,0X96,0XA4,0X30,0X00,0X00,0X0C,
0X7F,0XFF,0XB5,0XA4,0XB8,0X00,0X00,0X0C,0X7F,0XFF,0X35,0XA4,0XB8,0X00,0X00,0X0C,
0X7F,0XFF,0X25,0X26,0XB8,0X00,0X00,0X0C,0X7F,0XFF,0X2D,0X36,0X9C,0X00,0X00,0X00,
0X7F,0XFF,0X6D,0XB6,0XDC,0X00,0X07,0XFC,0X7F,0XFF,0X6D,0XB3,0XDC,0X00,0X00,0X0C,
0X7F,0XFF,0X6D,0X9B,0XDC,0X00,0X00,0X0C,0X7F,0XFF,0X2C,0XDB,0XDC,0X00,0X04,0X0C,
0X7F,0XFF,0X26,0XDB,0XD8,0X00,0X07,0X00,0X7F,0XFF,0X26,0XD9,0XD8,0X00,0X03,0XC0,
0X7F,0XFF,0XB6,0XC9,0X98,0X00,0X01,0XFC,0X3F,0XFF,0X96,0X4B,0XB0,0X00,0X03,0XC0,
0X3F,0XFF,0XC2,0X4A,0XB2,0X00,0X07,0X00,0X3F,0XFF,0XC2,0X5A,0XE6,0X00,0X04,0X00,
0X1F,0XFF,0XE2,0X5A,0XC7,0X00,0X00,0X00,0X1F,0XFF,0XF0,0X57,0X0F,0X00,0X07,0XFC,
0X0F,0XFF,0XF8,0X17,0X3F,0X00,0X04,0XCC,0X0F,0XFF,0XFE,0X00,0X7F,0X80,0X04,0XCC,
0X07,0XFF,0XFF,0XC1,0XFF,0X80,0X04,0XCC,0X07,0XFF,0XFF,0XFF,0XFF,0X80,0X04,0XCC,
0X03,0XFF,0XFF,0XFF,0XFF,0XC0,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XC0,0X07,0X98,
0X01,0XFF,0XFF,0XFF,0XFF,0XE0,0X07,0XDC,0X00,0XFF,0XFF,0XFF,0XFF,0XE0,0X0C,0XCC,
0X00,0X7F,0XFF,0XFF,0XFF,0XF0,0X06,0XCC,0X00,0X3F,0XFF,0XFF,0XFF,0XF8,0X06,0X7C,
0X00,0X0F,0XFF,0XFF,0XFF,0XF8,0X00,0X30,0X00,0X07,0XFF,0XFF,0XFF,0XFC,0X00,0X00,
0X00,0X01,0XFF,0XFF,0XFF,0XF8,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XE0,0X00,0X00,
0X00,0X00,0X3F,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X07,0XFF,0XFE,0X00,0X00,0X00,
0X00,0X00,0X00,0X3F,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,};








