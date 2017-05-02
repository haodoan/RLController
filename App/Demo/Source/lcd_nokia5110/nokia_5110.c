#include "nokia_5110.h"
#include "english_6x8_pixel.h"
#include "stm32f10x_lib.h"
#include "stm32f10x_gpio.h"
#include <stdio.h>
void LCD_init(void);
void LCD_clear(void);
void LCD_write_string(unsigned char X,unsigned char Y,char *s,unsigned char invert);
void LCD_write_char(unsigned char c);
void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,unsigned char *map,
                        unsigned char Pix_x,unsigned char Pix_y);
void LCD_write_byte(unsigned char dat, unsigned char dc);
void delay_1us(void);
void LCD_cursor(unsigned char X, unsigned char Y);
void LCD_write_char_invert(unsigned char c);
void LCD_write_number(unsigned char X,unsigned char Y,unsigned int number,unsigned char invert);
/*-----------------------------------------------------------------------
LCD_init          : 3310LCD��ʼ��

��д����          ��2004-8-10
����޸�����      ��2004-8-10
-----------------------------------------------------------------------*/

void delay_1us(void)                 //1us��ʱ����
{
    unsigned int i;
    for(i=0;i<1000;i++);
}

void delay_1m(void)                 //1ms��ʱ����
{
    unsigned int i;
    for (i=0;i<1140;i++);
}

void delay_nms(unsigned int n)       //N ms��ʱ����
{
    unsigned int i=0;
    for (i=0;i<n;i++)
        delay_1m();
}


void LCD_init(void)
{
    GPIO_WriteBit(GPIOC,GPIO_Pin_7,Bit_SET);//LCD_RST = 0;
    // ����һ����LCD��λ�ĵ͵�ƽ����
    GPIO_WriteBit(GPIO_LCD_RST_PORT,GPIO_LCD_RST,Bit_RESET);//LCD_RST = 0;
    delay_1us();

    GPIO_WriteBit(GPIO_LCD_RST_PORT,GPIO_LCD_RST,Bit_SET);//LCD_RST = 1;

    // �ر�LCD
    GPIO_WriteBit(GPIO_LCD_CE_PORT,GPIO_LCD_CE,Bit_RESET);//LCD_CE = 0;
    delay_1us();
    // ʹ��LCD
    GPIO_WriteBit(GPIO_LCD_CE_PORT,GPIO_LCD_CE,Bit_SET);//LCD_CE = 1;
    delay_1us();

    LCD_write_byte(0x21, 0);	// ʹ����չ��������LCDģʽ
    LCD_write_byte(0xc8, 0);	// ����ƫ�õ�ѹ
    LCD_write_byte(0x06, 0);	// �¶�У��
    LCD_write_byte(0x13, 0);	// 1:48
    LCD_write_byte(0x20, 0);	// ʹ�û�������
    LCD_clear();	        // ����
    LCD_write_byte(0x0c, 0);	// �趨��ʾģʽ��������ʾ

    // �ر�LCD
    GPIO_WriteBit(GPIO_LCD_CE_PORT,GPIO_LCD_CE,Bit_RESET);//LCD_CE = 0;
}

/*-----------------------------------------------------------------------
LCD_clear         : LCD��������

��д����          ��2004-8-10
����޸�����      ��2004-8-10
-----------------------------------------------------------------------*/
void LCD_clear(void)
{
    unsigned int i;

    LCD_write_byte(0x0c, 0);
    LCD_write_byte(0x80, 0);

    for (i=0; i<504; i++)
        LCD_write_byte(0, 1);
}

/*-----------------------------------------------------------------------
LCD_set_XY        : ����LCD���꺯��

���������X       ��0��83
          Y       ��0��5

��д����          ��2004-8-10
����޸�����      ��2004-8-10
-----------------------------------------------------------------------*/
void LCD_set_XY(unsigned char X, unsigned char Y)
{
    LCD_write_byte(0x80 | 6*Y, 0);		// column
    LCD_write_byte(0x40 | X, 0);          	// row
}

void LCD_cursor(unsigned char X, unsigned char Y)
{
    char space =' ';
    unsigned char line;

    space -=32;
    LCD_set_XY(X,Y);

    for (line=0; line<6; line++)
        LCD_write_byte(~font6x8[space][line], 1);
}

/*-----------------------------------------------------------------------
LCD_write_char    : ��ʾӢ���ַ�

���������c       ����ʾ���ַ���

��д����          ��2004-8-10
����޸�����      ��2004-8-10
-----------------------------------------------------------------------*/
void LCD_write_char(unsigned char c)
{
    unsigned char line;

    c -= 32;

    for (line=0; line<6; line++)
        LCD_write_byte(font6x8[c][line], 1);
}

void LCD_write_char_invert(unsigned char c)
{
    unsigned char line;

    c -= 32;

    for (line=0; line<6; line++)
        LCD_write_byte(~font6x8[c][line], 1);
}
/*-----------------------------------------------------------------------
LCD_write_String  : Ӣ���ַ�����ʾ����

���������*s      ��Ӣ���ַ���ָ�룻
          X��Y    : ��ʾ�ַ�����λ��,x 0-83 ,y 0-5

��д����          ��2004-8-10
����޸�����      ��2004-8-10
-----------------------------------------------------------------------*/
void LCD_write_string(unsigned char X,unsigned char Y,char *s,unsigned char invert)
{
    LCD_set_XY(X,Y);
    while (*s)
    {
        (invert == 0)?LCD_write_char(*s):LCD_write_char_invert(*s);
        s++;
    }
}

void LCD_write_number(unsigned char X,unsigned char Y, unsigned int number,unsigned char invert)
{
    char buff[5];

    sprintf(buff,"%d",number);
    LCD_set_XY(X,Y);
    LCD_write_string(X,Y,buff,invert);
}
/*-----------------------------------------------------------------------
LCD_draw_map      : λͼ���ƺ���

���������X��Y    ��λͼ���Ƶ���ʼX��Y���ꣻ
          *map    ��λͼ�������ݣ�
          Pix_x   ��λͼ���أ�����
          Pix_y   ��λͼ���أ���

��д����          ��2004-8-13
����޸�����      ��2004-8-13
-----------------------------------------------------------------------*/
void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,unsigned char *map,
                        unsigned char Pix_x,unsigned char Pix_y)
{
    unsigned int i,n;
    unsigned char row;

    if (Pix_y%8==0) row=Pix_y/8;      //����λͼ��ռ����
    else
        row=Pix_y/8+1;

    for (n=0;n<row;n++)
    {
      	LCD_set_XY(X,Y);
        for(i=0; i<Pix_x; i++)
        {
            LCD_write_byte(map[i+n*Pix_x], 1);
        }
        Y++;                         //����
    }
}

/*-----------------------------------------------------------------------
LCD_write_byte    : ʹ��SPI�ӿ�д���ݵ�LCD

���������data    ��д������ݣ�
          command ��д����/����ѡ��

��д����          ��2004-8-10
����޸�����      ��2004-8-13
-----------------------------------------------------------------------*/
void LCD_write_byte(unsigned char dat, unsigned char command)
{
    unsigned char i;
    GPIO_WriteBit(GPIO_LCD_CE_PORT,GPIO_LCD_CE,Bit_RESET);//LCD_CE = 0;

    if (command == 0)
        GPIO_WriteBit(GPIO_LCD_DC_PORT,GPIO_LCD_DC,Bit_RESET);//LCD_DC = 0;
    else
        GPIO_WriteBit(GPIO_LCD_DC_PORT,GPIO_LCD_DC,Bit_SET);//LCD_DC = 1;
    for(i=0;i<8;i++)
    {
        if(dat&0x80)
            GPIO_WriteBit(GPIO_SDIN_PORT,GPIO_SDIN,Bit_SET);//SDIN = 1;
        else
            GPIO_WriteBit(GPIO_SDIN_PORT,GPIO_SDIN,Bit_RESET);//SDIN = 0;
        GPIO_WriteBit(GPIO_SCLK_PORT,GPIO_SCLK,Bit_RESET);//SCLK = 0;
        dat = dat << 1;
        GPIO_WriteBit(GPIO_SCLK_PORT,GPIO_SCLK,Bit_SET);//SCLK = 1;
    }
    GPIO_WriteBit(GPIO_LCD_CE_PORT,GPIO_LCD_CE,Bit_SET);//LCD_CE = 1;
}


