#include "oled.h"
#include "fontupd.h"
#include "w25qxx.h"
#include "text.h"
#include "string.h"
#include "usart.h"

//code �ַ�ָ�뿪ʼ
//���ֿ��в��ҳ���ģ
//code �ַ����Ŀ�ʼ��ַ,GBK��
//mat  ���ݴ�ŵ�ַ (size/8+((size%8)?1:0))*(size) bytes��С
//size:�����С
void Get_HzMat(unsigned char *code, unsigned char *mat, u8 size)
{
    unsigned char qh, ql;
    unsigned char i;
    unsigned long foffset;
    u8 csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size); //�õ�����һ���ַ���Ӧ������ռ���ֽ���
    qh = *code;
    ql = *(++code);
    if (qh < 0x81 || ql < 0x40 || ql == 0xff || qh == 0xff) //�� ���ú���
    {
        for (i = 0; i < csize; i++)
            *mat++ = 0x00; //�������
        return;            //��������
    }
    if (ql < 0x7f)
        ql -= 0x40; //ע��!
    else
        ql -= 0x41;
    qh -= 0x81;
    foffset = ((unsigned long)190 * qh + ql) * csize; //�õ��ֿ��е��ֽ�ƫ����
    switch (size)
    {
    case 12:
        W25QXX_Read(mat, foffset + ftinfo.f12addr, csize);
        break;
    case 16:
        W25QXX_Read(mat, foffset + ftinfo.f16addr, csize);
        break;
    case 24:
        W25QXX_Read(mat, foffset + ftinfo.f24addr, csize);
        break;
    case 32:
        W25QXX_Read(mat, foffset + ftinfo.f32addr, csize);
        break;
    }
}
//��ʾһ��ָ����С�ĺ���
//x,y :���ֵ�����
//font:����GBK��
//size:�����С
//mode:0,������ʾ,1,������ʾ
void Show_Font(u16 x, u16 y, u16 xend, u16 yend, u16 size, u8 *font)
{
    u8 temp, t, pos, color = 1;
    u16 y0 = y;
    u8 dzk[128];
    u8 csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size); //�õ�����һ���ַ���Ӧ������ռ���ֽ���
    if (size != 12 && size != 16 && size != 24 && size != 32)
        return;                 //��֧�ֵ�size
    Get_HzMat(font, dzk, size); //�õ���Ӧ��С�ĵ�������
    for (pos = 0; pos < size * 2; pos++)
    {
        if (x > xend)
            break;       //�����յ�����
        temp = dzk[pos]; //�õ���������
        for (t = 0; t < 8; t++)
        {
            if (y <= yend)
            {
                if (temp & 0x80)
                    OLED_DrawPoint(x, y, color);
                else
                    OLED_DrawPoint(x, y, !color);
            }
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
