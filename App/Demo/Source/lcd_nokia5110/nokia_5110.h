#ifndef _NOKIA_5110_H_
#define _NOKIA_5110_H_


// pin define for n5110lcd_8key board
// change this if your hardware changed!
/* Library includes. */
#include "stm32f10x_lib.h"


#define LCD_WIDTH_PIXELS 84
#define NO_TEM "no template"


#define RCC_SCLK                                        RCC_APB2Periph_GPIOC		 /*����1ʹ�õ�GPIOʱ��*/
#define GPIO_SCLK_PORT                                  GPIOA    					 /*����1ʹ�õ�GPIO��*/
#define GPIO_SCLK                                       GPIO_Pin_5					 /*����1���ӵ�GPIO�ܽź�*/

#define RCC_SDIN                                        RCC_APB2Periph_GPIOC	     /*����2ʹ�õ�GPIOʱ��*/
#define GPIO_SDIN_PORT                                  GPIOA  						 /*����2ʹ�õ�GPIO��*/
#define GPIO_SDIN                                       GPIO_Pin_7				     /*����2���ӵ�GPIO�ܽź�*/

#define RCC_LCD_DC                                      RCC_APB2Periph_GPIOC		 /*����1ʹ�õ�GPIOʱ��*/
#define GPIO_LCD_DC_PORT                                GPIOC    					 /*����1ʹ�õ�GPIO��*/
#define GPIO_LCD_DC                                     GPIO_Pin_8					 /*����1���ӵ�GPIO�ܽź�*/

#define RCC_LCD_CE                                      RCC_APB2Periph_GPIOC	     /*����2ʹ�õ�GPIOʱ��*/
#define GPIO_LCD_CE_PORT                                GPIOA  						 /*����2ʹ�õ�GPIO��*/
#define GPIO_LCD_CE                                     GPIO_Pin_11				     /*����2���ӵ�GPIO�ܽź�*/

#define RCC_LCD_RST                                     RCC_APB2Periph_GPIOC	     /*����2ʹ�õ�GPIOʱ��*/
#define GPIO_LCD_RST_PORT                               GPIOA  						 /*����2ʹ�õ�GPIO��*/
#define GPIO_LCD_RST                                    GPIO_Pin_12				     /*����2���ӵ�GPIO�ܽź�*/



extern void LCD_init(void);
extern void LCD_clear(void);
extern void LCD_write_string(unsigned char X,unsigned char Y,char *s,unsigned char invert);
extern void LCD_write_char(unsigned char c);
extern void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,unsigned char *map,
                               unsigned char Pix_x,unsigned char Pix_y);
extern void LCD_write_byte(unsigned char dat, unsigned char dc);
extern void delay_1us(void);
extern void LCD_cursor(unsigned char X, unsigned char Y);
extern void LCD_write_char_invert(unsigned char c);
extern void LCD_write_number(unsigned char X,unsigned char Y,unsigned int number,unsigned char invert);
void delay_1m(void)  ;
void delay_nms(unsigned int n);
void LCD_set_XY(unsigned char X, unsigned char Y);

#endif

