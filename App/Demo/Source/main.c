/*
	FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
	All rights reserved

	VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

	This file is part of the FreeRTOS distribution.

	FreeRTOS is free software; you can redistribute it and/or modify it under
	the terms of the GNU General Public License (version 2) as published by the
	Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

	***************************************************************************
	>>!   NOTE: The modification to the GPL is included to allow you to     !<<
	>>!   distribute a combined work that includes FreeRTOS without being   !<<
	>>!   obliged to provide the source code for proprietary components     !<<
	>>!   outside of the FreeRTOS kernel.                                   !<<
	***************************************************************************

	FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
	FOR A PARTICULAR PURPOSE.  Full license text is available on the following
	link: http://www.freertos.org/a00114.html

	***************************************************************************
	 *                                                                       *
	 *    FreeRTOS provides completely free yet professionally developed,    *
	 *    robust, strictly quality controlled, supported, and cross          *
	 *    platform software that is more than just the market leader, it     *
	 *    is the industry's de facto standard.                               *
	 *                                                                       *
	 *    Help yourself get started quickly while simultaneously helping     *
	 *    to support the FreeRTOS project by purchasing a FreeRTOS           *
	 *    tutorial book, reference manual, or both:                          *
	 *    http://www.FreeRTOS.org/Documentation                              *
	 *                                                                       *
	***************************************************************************

	http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
	the FAQ page "My application does not run, what could be wrong?".  Have you
	defined configASSERT()?

	http://www.FreeRTOS.org/support - In return for receiving this top quality
	embedded software for free we request you assist our global community by
	participating in the support forum.

	http://www.FreeRTOS.org/training - Investing in training allows your team to
	be as productive as possible as early as possible.  Now you can receive
	FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
	Ltd, and the world's leading authority on the world's leading RTOS.

	http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
	including FreeRTOS+Trace - an indispensable productivity tool, a DOS
	compatible FAT file system, and our tiny thread aware UDP/IP stack.

	http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
	Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

	http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
	Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
	licenses offer ticketed support, indemnification and commercial middleware.

	http://www.SafeRTOS.com - High Integrity Systems also provide a safety
	engineered and independently SIL3 certified version for use in safety and
	mission critical applications that require provable dependability.

	1 tab == 4 spaces!
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the LCD by the 'Check' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" task -  This only executes every five seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write an error to the LCD (via the LCD task).  If all the demo tasks are
 * executing with their expected behaviour then the check task writes PASS
 * along with the max jitter time to the LCD (again via the LCD task), as
 ":.k, * described above.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
/* Scheduler includes. */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"

/* Demo app includes. */
#include "serial.h"
#include "flasheeprom.h"
#include "lcd_nokia5110/nokia_5110.h"
/* Task priorities. */
#define mainRELAY_TASK_PRIORITY     (tskIDLE_PRIORITY + 3)
#define mainKEYBOARD_TASK_PRIORITY  (tskIDLE_PRIORITY + 4)
#define mainLED_TASK_PRIORITY       (tskIDLE_PRIORITY)
/* The check task uses the sprintf function so requires a little more stack. */
#define mainRELAY_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE + 128)
#define mainKEYBOARD_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE + 128)
#define mainLED_TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE)
/* The time between cycles of the 'check' task. */
#define mainGPS_DELAY ((TickType_t)5000 / portTICK_PERIOD_MS)
#define mainLED_BLINK_DELAY (500)


#define RELAY_BLOCK_TIME 5000
#define GPRS_BUFFER_SIZE 200

#define RL_NUMBER_MAX   12

#define LOCK_FLASH(x)   xSemaphoreTake( x, ( TickType_t ) portMAX_DELAY )
#define UNLOCK_FLASH(x) xSemaphoreGive( x );
typedef struct RELAY_Output
{
	GPIO_TypeDef*    port;
	u16             pin;
}RELAY_Output_t;

typedef struct KEYBOARD_Input
{
	GPIO_TypeDef *   port;
	u16             pin;
}KEYBOARD_Input_t;

typedef struct TimerSetting
{
	unsigned int uiTON;
	unsigned int uiTOFF;
	unsigned int uiUNIT[2];
}TimerSetting_t;

RELAY_Output_t RELAY_OUTPUT_MAPPING[] = 
{
	{GPIOB, GPIO_Pin_0},
	{GPIOB, GPIO_Pin_1},
	{GPIOB, GPIO_Pin_5},
	{GPIOB, GPIO_Pin_6},
	{GPIOB, GPIO_Pin_7},
	{GPIOB, GPIO_Pin_8},
	{GPIOB, GPIO_Pin_9},
	{GPIOB, GPIO_Pin_10},
	{GPIOB, GPIO_Pin_11},
	{GPIOB, GPIO_Pin_12},
	{GPIOB, GPIO_Pin_13},
	{GPIOB, GPIO_Pin_14},    
};

KEYBOARD_Input_t KEY_GPIO_MAPPING[] = 
{
	{GPIOA, GPIO_Pin_0},
	{GPIOA, GPIO_Pin_2},
	{GPIOA, GPIO_Pin_1}    
};

enum
{
	KEY_UP = 0,
	KEY_DOWN,
	KEY_ENTER,
	KEY_MAX
};

enum
{
	TON_SET = 0,
	TOFF_SET,
	OK_SET,
	MAX_SET
};
enum
{
	THOUS,
	HUNDERD,
	TENTH,
	UNITS,
	UNIT,
	MAX_NUM
};

/*-----------------------------------------------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware(void);

/*
 * The LCD is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the LCD directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
/*
 * Retargets the C library printf function to the USART.
 */
int fputc(int ch, FILE *f);

/*
 * Checks the status of all the demo tasks then prints a message to the
 * display.  The message will be either PASS - and include in brackets the
 * maximum measured jitter time (as described at the to of the file), or a
 * message that describes which of the standard demo tasks an error has been
 * discovered in.
 *
 * Messages are not written directly to the terminal, but passed to vLCDTask
 * via a queue.
 */
static void vKeyboardTask(void *pvParameters);
static void vLCDTask(void *pvParameters);
static void vRelayTask(void *pvParameters);
u16 FindKeyBoard(u8);
void ChangingNumber(u8 *number, u8 key);
void DisplayCursorStr(int row);
/*
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
extern void vSetupTimerTest(void);
void error_lcd_printf(void);

/*handler for using USART*/

QueueHandle_t qTLTimeOnOff;
SemaphoreHandle_t FLASH_Mutex;
/*-----------------------------------------------------------*/

int main(void)
{
#ifdef DEBUG
	debug();
#endif

	prvSetupHardware();

	LCD_init();
	LCD_clear();

	LCD_write_string(0, 0, "SETTING TIME",1);
 
	qTLTimeOnOff = xQueueCreate(20, sizeof(TimerSetting_t));
	if (qTLTimeOnOff == NULL)
	{
		LCD_write_string(0, 3, "QUEUE MEM fAIL",0);
		while (TRUE);
	}
	FLASH_Mutex = xSemaphoreCreateMutex();
	if( FLASH_Mutex == NULL )
	{
		LCD_write_string(0, 3, "MUTEX MEM fAIL",0);
		while(1);
	}

	/* Start the tasks defined within this file/specific to this demo. */
	// xTaskCreate( vGPSTask, "GPS", mainGPS_TASK_STACK_SIZE, NULL, mainGPS_TASK_PRIORITY, NULL );
	xTaskCreate(vKeyboardTask, "KEYBOARD", mainKEYBOARD_TASK_STACK_SIZE, NULL, mainKEYBOARD_TASK_PRIORITY, NULL);
	xTaskCreate(vRelayTask, "RELAY", mainRELAY_TASK_STACK_SIZE, NULL, mainRELAY_TASK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}
/*-----------------------------------------------------------*/
/*Keyboard task*/
static void vKeyboardTask(void *pvParameters)
{
	static u8 row_offset = 0;
	static u8 num_offset = 0;
	static u8 KeyEnter = 0;
	static u8 nghin[2],tram[2],chuc[2],dv[2];
	//static u8 time_unit[2] = 0;
	u8 key;
	u8 SettingAccept = TRUE;
	TimerSetting_t stTimeSet = {1000,1000};
	char buff[5] = {0};
	char buff_lcd[16] = {0};
		
	//Read setting from FLASH
	//Read from FLASH
		
	LOCK_FLASH(FLASH_Mutex);
	FlashReadEEprom(&stTimeSet,sizeof(TimerSetting_t));
	UNLOCK_FLASH(FLASH_Mutex);

	if(stTimeSet.uiTON == 0xFFFFFFFF)
	{
		stTimeSet.uiTON = 1000;
	}
	if(stTimeSet.uiTOFF == 0xFFFFFFFF)
	{
		stTimeSet.uiTOFF = 1000;
	}
	if(stTimeSet.uiUNIT[0] == 0xFFFFFFFF)
	{
		stTimeSet.uiUNIT[0]  = 0;
	}
	if(stTimeSet.uiUNIT[1] == 0xFFFFFFFF)
	{
		stTimeSet.uiUNIT[1] = 0;
	}	
	if(stTimeSet.uiUNIT[0] == 1)//second
	{
		stTimeSet.uiTON = stTimeSet.uiTON/1000;
		sprintf(buff_lcd,"TON  :%04d s ",stTimeSet.uiTON);
		LCD_write_string(2, 0, buff_lcd,0);
	}
	else
	{
		sprintf(buff_lcd,"TON  :%04d ms",stTimeSet.uiTON);
		LCD_write_string(2, 0, buff_lcd,0);		
	}
	if(stTimeSet.uiUNIT[1] == 1)
	{
		stTimeSet.uiTOFF = stTimeSet.uiTOFF/1000;
		sprintf(buff_lcd,"TOFF :%04d s ",stTimeSet.uiTOFF);
		LCD_write_string(3, 0, buff_lcd,0);
	}
	else
	{
		sprintf(buff_lcd,"TOFF :%04d ms",stTimeSet.uiTOFF);
		LCD_write_string(3, 0, buff_lcd,0);		
	}

	LCD_write_string(4, 6, "OK",0);


	nghin[0] = stTimeSet.uiTON/1000;
	tram[0] = (stTimeSet.uiTON/100)%10;
	chuc[0] = (stTimeSet.uiTON%100)/10;
	dv[0] = (stTimeSet.uiTON%100)%10;

	nghin[1] = stTimeSet.uiTOFF/1000;
	tram[1] = (stTimeSet.uiTOFF/100)%10;
	chuc[1] = (stTimeSet.uiTOFF%100)/10;
	dv[1] = (stTimeSet.uiTOFF%100)%10;	

	//Convert second to msecond
	if(stTimeSet.uiUNIT[0]) stTimeSet.uiTON = stTimeSet.uiTON*1000;
	if(stTimeSet.uiUNIT[1]) stTimeSet.uiTOFF = stTimeSet.uiTOFF*1000;
        
        xQueueSend(qTLTimeOnOff,&stTimeSet,10);
	while(TRUE)
	{
		/* Read key pressed */
		do
		{
			key = FindKeyBoard(0);
			vTaskDelay(100);  
		}while(KEY_MAX == key);

		if(KeyEnter == 0)
		{
			if(KEY_ENTER == key)
			{
				KeyEnter = 2;
				key  = KEY_MAX;
				LCD_write_number(row_offset + 2,6,nghin[row_offset],1);                
			}       
		}

		else if(KeyEnter == 2)
		{
			//char buff[5] = {0};
			if(row_offset > 1 ) {LCD_write_string(5,0,"ERROR",1);continue;}
			switch(num_offset)
			{
				case THOUS:
					ChangingNumber(&nghin[row_offset],key);
					LCD_write_number(row_offset + 2,6,nghin[row_offset],1);
					if(KEY_ENTER == key)
					{
						LCD_write_number(row_offset + 2,6,nghin[row_offset],0);
						LCD_write_number(row_offset + 2,7,tram[row_offset],1);
					}                    
					break;
				case HUNDERD:
					ChangingNumber(&tram[row_offset],key);
					LCD_write_number(row_offset + 2,7,tram[row_offset],1);
					if(KEY_ENTER == key)
					{
						LCD_write_number(row_offset + 2,7,tram[row_offset],0);
						LCD_write_number(row_offset + 2,8,chuc[row_offset],1);
					}
					break;
				case TENTH:
					ChangingNumber(&chuc[row_offset],key);
					LCD_write_number(row_offset + 2,8,chuc[row_offset],1);
					if(KEY_ENTER == key)
					{
						//LCD_write_string(row_offset + 2, 8, buff,0);
						LCD_write_number(row_offset + 2,8,chuc[row_offset],0);
						LCD_write_number(row_offset + 2,9,dv[row_offset],1);                        
					}                    
					break;
				case UNITS:
					ChangingNumber(&dv[row_offset],key);
					LCD_write_number(row_offset + 2,9,dv[row_offset],1);
					if(KEY_ENTER == key)
					{
						LCD_write_number(row_offset + 2,9,dv[row_offset],0);
						stTimeSet.uiUNIT[row_offset] ? sprintf(buff,"s "):sprintf(buff,"ms");                        
						LCD_write_string(row_offset + 2, 11, buff,1);                                                 
					}                                        
					break;
				case UNIT:
					stTimeSet.uiUNIT[row_offset] ? sprintf(buff,"s "):sprintf(buff,"ms");                        
					LCD_write_string(row_offset + 2, 11, buff,1);                         
					if(KEY_UP == key)
					{
						stTimeSet.uiUNIT[row_offset] = !stTimeSet.uiUNIT[row_offset];
						stTimeSet.uiUNIT[row_offset] ? sprintf(buff,"s "):sprintf(buff,"ms");            
						LCD_write_string(row_offset + 2, 11, buff,1); 

					}
					if(KEY_ENTER == key)
					{
						stTimeSet.uiUNIT[row_offset] ? sprintf(buff,"s "):sprintf(buff,"ms");            
						LCD_write_string(row_offset + 2, 11, buff,0); 

					}                    
					break;                                                                
			}
			if(key == KEY_ENTER) 
			{                
				if(num_offset++ == MAX_NUM -1)
				{
					//KeyEnter = 1;
					num_offset = 0;
					if(row_offset == 0)
					{
						 stTimeSet.uiTON = nghin[row_offset]*1000+ tram[row_offset]*100 + chuc[row_offset]*10 + dv[row_offset];
						 if(stTimeSet.uiUNIT[row_offset]) stTimeSet.uiTON = stTimeSet.uiTON*1000;
						 LCD_write_number(3,6,nghin[1],1);
						 //KeyEnter = 0;
					}
					else if(row_offset == 1)
					{
						 stTimeSet.uiTOFF = nghin[row_offset]*1000+ tram[row_offset]*100 + chuc[row_offset]*10 + dv[row_offset];
						 if(stTimeSet.uiUNIT[row_offset]) stTimeSet.uiTOFF = stTimeSet.uiTOFF*1000;
						 KeyEnter = 3;
						 key = KEY_MAX;
						 
					}
					row_offset++;                    
				 }               
			   
			}
		}
		if(KeyEnter == 3)
		{
			LCD_write_string(4, 6, "OK",1);
			LCD_write_string(4, 8, "   ",0);
			switch(key)
			{
				case KEY_UP :
						//LCD print OK
					LCD_write_string(4, 6, "OK",1);
					LCD_write_string(4, 8, "   ",0);                   
					SettingAccept = TRUE;
					break;
				case KEY_DOWN:
					////LCD print CACEL
					LCD_write_string(4, 6, "CACEL",1);
					SettingAccept = FALSE;
					break;
				case KEY_ENTER:
					if(SettingAccept)
					{
						LCD_write_string(4, 6, "OK  ",0);
						KeyEnter = 0;
						SettingAccept = FALSE;
						LOCK_FLASH(FLASH_Mutex);
						FlashWriteEEprom(&stTimeSet,sizeof(TimerSetting_t));
						UNLOCK_FLASH(FLASH_Mutex);
						// Send message queue Time setting to RELAY task
						xQueueSend(qTLTimeOnOff,&stTimeSet,10);
					}
					else
					{
						LCD_write_string(4, 6, "OK  ",0);
						KeyEnter = 0;
						//SettingAccept = FALSE;
					}
					//Send offset
					//
					break;
			}
			row_offset  = 0;
		} 
		//vTaskDelay(100);   
	}
}

/*RELAY task*/
static void vRelayTask(void *pvParameters)
{
	TimerSetting_t stTimeset;
	int rl_scan_idx;
	char rl_status[16] = {0};
	while(TRUE)
	{
		if(pdTRUE != xQueueReceive( qTLTimeOnOff, &stTimeset, ( TickType_t ) 5 ))
		{
			//Read from FLASH
			LOCK_FLASH(FLASH_Mutex);
			FlashReadEEprom(&stTimeset,sizeof(TimerSetting_t));
			UNLOCK_FLASH(FLASH_Mutex);
		}
		for(rl_scan_idx = 0;rl_scan_idx < RL_NUMBER_MAX;rl_scan_idx++)
		{
			sprintf(rl_status,"Relay %d ON  ",rl_scan_idx + 1); 
			LCD_write_string(5, 0, rl_status,0);			
			GPIO_WriteBit(RELAY_OUTPUT_MAPPING[rl_scan_idx].port,RELAY_OUTPUT_MAPPING[rl_scan_idx].pin, Bit_RESET);
			vTaskDelay(stTimeset.uiTON);
			sprintf(rl_status,"Relay %d OFF ",rl_scan_idx + 1); 
			LCD_write_string(5, 0, rl_status,0);			
			GPIO_WriteBit(RELAY_OUTPUT_MAPPING[rl_scan_idx].port,RELAY_OUTPUT_MAPPING[rl_scan_idx].pin, Bit_SET);
			vTaskDelay(stTimeset.uiTOFF);             
		}        
	}
}
/*LCD task*/
static void vLCDTask(void *pvParameters)
{
	// int LCD here
	while(TRUE)
	{
		//if( xQueueReceive( qTLTimeOnOff, &stTimeset, ( TickType_t ) 5 ) )
		{

		}
	}
}
/*-----------------------------------------------------------*/

u16 FindKeyBoard(u8 keepingKEY)
{
	u8 idx_key;

	for(idx_key = 0 ; idx_key < KEY_MAX;idx_key++)
	{
		if(Bit_RESET == GPIO_ReadInputDataBit(KEY_GPIO_MAPPING[idx_key].port,KEY_GPIO_MAPPING[idx_key].pin))
		{
			//if(!keepingKEY)
			{
				while(Bit_RESET == GPIO_ReadInputDataBit(KEY_GPIO_MAPPING[idx_key].port,KEY_GPIO_MAPPING[idx_key].pin));    
			}            
			return idx_key;
		}
	}
	return KEY_MAX;
}


void ChangingNumber(u8 *number,u8 key)
{
	u8 tmp_num = *number;
	if(KEY_UP == key)
	{
		if(tmp_num++ == 9)
		{
			tmp_num = 0;
		}
	}
	else if(KEY_DOWN == key)
	{
		if( tmp_num-- == 0)
		{
			tmp_num = 9;
		}
	}
	else if(KEY_ENTER == key)
	{

	}
	*number = tmp_num;
}

static void prvSetupHardware(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready. */
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
	{
	}

	/* 2 wait states required on the flash. */
	*((unsigned long *)0x40022000) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);

	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1);

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config(RCC_HCLK_Div2);

	/* PLLCLK = 12MHz * 6 = 72 MHz. */
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);

	/* Enable PLL. */
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready. */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source. */
	while (RCC_GetSYSCLKSource() != 0x08)
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO,
						   ENABLE);

	/* SPI2 Periph clock enable */
	// RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

  //  xSerialPortInitMinimal(USART2, &uart2_handle, 115200, MAX_LENGH_STR);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*Initial for BL*/
	/*Initial for lcd AND BL*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;// &(GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4) ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// vParTestInitialise();
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_Write(GPIOB, 0xFFFF);
}

/*-----------------------------------------------------------*/

void error_lcd_printf()
{
	LCD_write_string(0, 3, "FAIL...",0);
	while (TRUE)
		;
}
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
   // xSerialPutChar(&uart2_handle, ch, 0);

	return ch;
}
/*-----------------------------------------------------------*/

#ifdef DEBUG
/* Keep the linker happy. */
void assert_failed(unsigned char *pcFile, unsigned long ulLine)
{
	for (;;)
	{
	}
}
#endif
