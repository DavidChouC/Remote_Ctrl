#include "KunUI.h"

/**
  *@breif      ��������
  *@param      x ���Ͻǵ�x����
	*@param			 y ���Ͻǵ�y����
	*@param			 w �������ܿ���
	*@param			 y �������ܸ߶�
  *@retval     none
  */
void Draw_ProgressDialog(u8g2_t *u8g2 , u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h)
{
	u8g2_ClearBuffer(u8g2);
	for(uint8_t i=x;i<w;i++)
	{
		u8g2_DrawBox(u8g2,x,y,i,h);
		u8g2_SendBuffer(u8g2);
		HAL_Delay(50);
	}
}


const unsigned char Kun_1[]=
{
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x4F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x3D,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x80,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xA0,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x38,0xBF,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x7C,0xDE,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFE,0xCE,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xFF,0xE6,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0xFF,0xF1,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x80,0xFF,0xF9,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFF,0xFD,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x80,0xFF,0xFD,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x80,0xFF,0xFD,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFE,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xFE,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFC,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xA8,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA4,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x54,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x48,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xB4,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xC8,0x56,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xBA,0x2D,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x54,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x2A,0x54,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x1A,0xA8,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x15,0x50,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x0A,0x50,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x05,0xA0,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x0D,0x40,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x80,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x05,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x80,0x02,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0xF0,0xFB,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x00,0x0C,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x80,0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x80,0x01,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x80,0x01,0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0xC0,0x03,0x00,0x38,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0xF8,0x03,0x00,0x74,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,/*D:\Users\Administrator\Desktop\Competition\kun_gif\image001.bmp*/0

};

const unsigned char Kun_2[]=
{
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x2F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xA0,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xD0,0x05,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xF0,0x0F,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x78,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x40,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x60,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xF0,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0x3C,0x03,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0x9D,0x03,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0xCB,0x03,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF8,0xE3,0x03,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFC,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xFC,0xFB,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFC,0xF7,0x03,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF8,0xFB,0x03,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0xF3,0x03,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0xFB,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF0,0xFB,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0xFB,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xA0,0x6A,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x40,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xA8,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x40,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA0,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xD0,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x54,0xAF,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0x58,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x6A,0xA8,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x14,0xA0,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x15,0x50,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x0A,0xA0,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x05,0x40,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x80,0x0A,0x80,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x06,0x80,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x80,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x80,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x80,0x02,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0xF0,0xFB,
	0xFF,0x0F,0x00,0x00,0x00,0x40,0x01,0x00,0x14,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,0xFF,0x0F,0x00,0x00,0x00,0x80,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,
	0xFF,0x07,0x00,0x00,0x00,0x40,0x01,0x00,0x28,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x80,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0xA0,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0xC0,0x00,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x60,0x00,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0xE0,0x00,0x00,0xE0,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0xF0,0x00,0x00,0xE0,0x01,0x00,0x00,0x00,0x00,0xF0,0xFF,/*D:\Users\Administrator\Desktop\Competition\kun_gif\image002.bmp*/0
};

const unsigned char Kun_3[]=
{
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x2F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xE0,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xC0,0x0F,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xF0,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0x06,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x80,0xFB,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xC0,0xFB,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xC0,0xFB,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xFB,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xE0,0xF3,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF0,0xF7,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0xF7,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF0,0x17,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0xEF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF8,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF0,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xF0,0xF7,0x0F,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xF7,0x0F,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xE0,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xC0,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xC0,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x40,0xD5,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xA0,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x40,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x50,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x80,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x50,0x55,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA0,0xAA,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0x7E,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x50,0xA3,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xA8,0x60,0x05,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x54,0x80,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x2A,0x80,0x0A,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x34,0x80,0x0A,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x0A,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x15,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x0A,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x05,0x00,0x14,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x04,0x00,0x28,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x80,0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x02,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0xF0,0xFB,
	0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,
	0xFF,0x07,0x00,0x00,0x00,0x80,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x40,0x00,0x00,0xA0,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0xC0,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0xC0,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0xC0,0x00,0x00,0x80,0x01,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0xF0,0x00,0x00,0xC0,0x03,0x00,0x00,0x00,0x00,0xF0,0xFF,/*D:\Users\Administrator\Desktop\Competition\kun_gif\image003.bmp*/0
};

const unsigned char Kun_4[]=
{
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x4F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x40,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xF0,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x70,0x03,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x80,0x79,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0xC0,0xFB,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xFB,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xF3,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0xF7,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0x17,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF8,0xE7,0x03,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFC,0xF7,0x03,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xFC,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xFC,0xF7,0x03,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xFC,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0xC0,0xF7,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x80,0xEA,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA0,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x50,0x55,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x80,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x50,0x55,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA0,0x6A,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x50,0xAD,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0xD6,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x54,0xA1,0x02,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0xC0,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x54,0x80,0x0A,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x32,0x80,0x05,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x0D,0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x14,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x0D,0x00,0x14,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x02,0x00,0x0C,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x05,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x04,0x00,0x28,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x80,0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x02,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0xF0,0xFB,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x80,0x00,0x00,0xA0,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0xC0,0x01,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0xC0,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0xC0,0x00,0x00,0x80,0x01,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0xF0,0x01,0x00,0xC0,0x03,0x00,0x00,0x00,0x00,0xF0,0xFF,/*D:\Users\Administrator\Desktop\Competition\kun_gif\image004.bmp*/0
};

const unsigned char Kun_5[]=
{
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x4F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x40,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xE0,0x03,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0x06,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0xF3,0x06,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x80,0xF7,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xC0,0xE7,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xEF,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0x6F,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0x1F,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xB8,0x9F,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xBC,0xDF,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x9E,0xCF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x9C,0xDF,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xF8,0xEF,0x0F,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xF0,0xEF,0x0F,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0xEF,0x0F,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xEF,0x0F,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xC0,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x20,0xD5,0x07,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0xA0,0xAA,0x03,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x40,0x55,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x80,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x50,0x55,0x01,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xA0,0x6A,0x01,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0xB6,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0xD4,0x6A,0x05,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0xA8,0xA0,0x05,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x6A,0x40,0x05,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x14,0x80,0x05,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x15,0x00,0x15,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0x00,0x0D,0x00,0x16,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x05,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x02,0x00,0x2C,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x02,0x00,0x28,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0x00,0x01,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x80,0x02,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0xF0,0xFB,
	0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,0xFF,0x0F,0x00,0x00,0x00,0x80,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0xF0,0xF5,
	0xFF,0x07,0x00,0x00,0x00,0x40,0x01,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,0x01,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x07,0x00,0x00,0x00,0xC0,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x0F,0x00,0x00,0x00,0xC0,0x00,0x00,0xC0,0x01,0x00,0x00,0x00,0x00,0xE0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0x40,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0xF0,0xFF,0xFF,0x07,0x00,0x00,0x00,0xE0,0x01,0x00,0x80,0x01,0x00,0x00,0x00,0x00,0xF0,0xFF,
	0xFF,0x0F,0x00,0x00,0x00,0xF0,0x01,0x00,0xC0,0x03,0x00,0x00,0x00,0x00,0xF0,0xFF,/*D:\Users\Administrator\Desktop\Competition\kun_gif\image005.bmp*/0

};
