#ifndef _NOKIA_5110_H_
#define _NOKIA_5110_H_


// pin define for n5110lcd_8key board
// change this if your hardware changed!
/* Library includes. */
#include "stm32f10x_lib.h"


#define LCD_WIDTH_PIXELS 84
#define NO_TEM "no template"


#define RCC_SCLK                                        RCC_APB2Periph_GPIOC		 /*按键1使用的GPIO时钟*/
#define GPIO_SCLK_PORT                                  GPIOA    					 /*按键1使用的GPIO组*/
#define GPIO_SCLK                                       GPIO_Pin_5					 /*按键1连接的GPIO管脚号*/

#define RCC_SDIN                                        RCC_APB2Periph_GPIOC	     /*按键2使用的GPIO时钟*/
#define GPIO_SDIN_PORT                                  GPIOA  						 /*按键2使用的GPIO组*/
#define GPIO_SDIN                                       GPIO_Pin_7				     /*按键2连接的GPIO管脚号*/

#define RCC_LCD_DC                                      RCC_APB2Periph_GPIOC		 /*按键1使用的GPIO时钟*/
#define GPIO_LCD_DC_PORT                                GPIOC    					 /*按键1使用的GPIO组*/
#define GPIO_LCD_DC                                     GPIO_Pin_8					 /*按键1连接的GPIO管脚号*/

#define RCC_LCD_CE                                      RCC_APB2Periph_GPIOC	     /*按键2使用的GPIO时钟*/
#define GPIO_LCD_CE_PORT                                GPIOA  						 /*按键2使用的GPIO组*/
#define GPIO_LCD_CE                                     GPIO_Pin_11				     /*按键2连接的GPIO管脚号*/

#define RCC_LCD_RST                                     RCC_APB2Periph_GPIOC	     /*按键2使用的GPIO时钟*/
#define GPIO_LCD_RST_PORT                               GPIOA  						 /*按键2使用的GPIO组*/
#define GPIO_LCD_RST                                    GPIO_Pin_12				     /*按键2连接的GPIO管脚号*/



extern void LCD_init(void);
extern void LCD_clear(void);
extern void LCD_write_string(unsigned char X,unsigned char Y,char *s);
extern void LCD_write_char(unsigned char c);
extern void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,unsigned char *map,
                               unsigned char Pix_x,unsigned char Pix_y);
extern void LCD_write_byte(unsigned char dat, unsigned char dc);
extern void delay_1us(void);
void delay_1m(void)  ;
void delay_nms(unsigned int n);
void LCD_set_XY(unsigned char X, unsigned char Y);

#endif

