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
#include "sim908.h"
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
}TimerSetting_t;

RELAY_Output_t RELAY_OUTPUT_MAPPING[] = 
{
    {GPIOB, GPIO_Pin_0},
    {GPIOB, GPIO_Pin_1},
    {GPIOB, GPIO_Pin_2},
    {GPIOB, GPIO_Pin_3},
    {GPIOB, GPIO_Pin_4},
    {GPIOB, GPIO_Pin_5},
    {GPIOB, GPIO_Pin_6},
    {GPIOB, GPIO_Pin_7},
    {GPIOB, GPIO_Pin_8},
    {GPIOB, GPIO_Pin_9},
    {GPIOB, GPIO_Pin_10},
    {GPIOB, GPIO_Pin_11},
    {GPIOB, GPIO_Pin_12},
};

KEYBOARD_Input_t KEY_GPIO_MAPPING[] = 
{
    {GPIOC, GPIO_Pin_0},
    {GPIOC, GPIO_Pin_2},
    {GPIOC, GPIO_Pin_4}    
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
SemaphoreHandle_t SIM908_Mutex;
/*-----------------------------------------------------------*/

int main(void)
{
#ifdef DEBUG
    debug();
#endif

    prvSetupHardware();

    /* Start the standard demo tasks. */
    //	vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
    //	vCreateBlockTimeTasks();
    //	vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
    //	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
    //	vStartIntegerMathTasks( mainINTEGER_TASK_PRIORITY );
    //	vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
    //	vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );

    LCD_init();
    LCD_clear();
    //LCD_write_string(0, 2, "    DEMO      ");
    //LCD_write_string(0, 3, "Controlling");
    //LCD_cursor(0,0);
    //LCD_write_string(0, 0, "1234567890123456");
    LCD_write_string(0, 0, "SETTING TIME",0);
    LCD_write_string(2, 0, "TON  :1234 ms",0);
    LCD_write_string(3, 0, "TOFF :5678 ms",0);
    LCD_write_string(4, 6, "OK",0);
 
    qTLTimeOnOff = xQueueCreate(20, sizeof(TimerSetting_t));
    if (qTLTimeOnOff == NULL)
    {
        LCD_write_string(0, 3, "QUEUE MEM fAIL",0);
        while (TRUE);
    }
    SIM908_Mutex = xSemaphoreCreateMutex();
    if( SIM908_Mutex == NULL )
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
    static u8 nghin,tram,chuc,dv,unit;
    static u8 time_unit = 0;
    u8 key;
    TimerSetting_t stTimeSet;
    char buff[5] = {0};
    while(TRUE)
    {
        //LCD_write_string(2 + row_offset, num_offset + 6, buff,1);
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
            }       
        }

        else if(KeyEnter == 1)
        {
            DisplayCursorStr(row_offset);          
            switch(key)
            {
                case KEY_UP :
                    if(row_offset++ == MAX_SET) row_offset = 0;
                    break;
                case KEY_DOWN:
                    if(row_offset-- == 0) row_offset = MAX_SET;
                    break;
                case KEY_ENTER:
                    //Send offset
                    KeyEnter = 2;
                    key = KEY_MAX;
                    break;
            }

            //continue;
        }
        else if(KeyEnter == 2)
        {
            //char buff[5] = {0};
            switch(num_offset)
            {
                case THOUS:
                    ChangingNumber(&nghin,key);
                    sprintf(buff,"%d",nghin);
                    LCD_write_string(row_offset + 2, 6, buff,1);
                    if(KEY_ENTER == key)
                    {
                        LCD_write_string(2, 6, buff,0);
                    }                    
                    break;
                case HUNDERD:
                    ChangingNumber(&tram,key);
                    sprintf(buff,"%d",tram);
                    LCD_write_string(row_offset + 2, 7, buff,1);
                    if(KEY_ENTER == key)
                    {
                        LCD_write_string(row_offset + 2, 7, buff,0);
                    }
                    break;
                case TENTH:
                    ChangingNumber(&chuc,key);
                    sprintf(buff,"%d",chuc);
                    LCD_write_string(row_offset + 2, 8, buff,1);
                    if(KEY_ENTER == key)
                    {
                        LCD_write_string(row_offset + 2, 8, buff,0);
                    }                    
                    break;
                case UNITS:
                    ChangingNumber(&dv,key);
                    sprintf(buff,"%d",dv);
                    LCD_write_string(row_offset + 2, 9, buff,1);
                    if(KEY_ENTER == key)
                    {
                        LCD_write_string(row_offset + 2, 9, buff,0);
                    }                                        
                    break;
                case UNIT:
                    time_unit ? sprintf(buff,"ms"):sprintf(buff," s");                        
                    LCD_write_string(row_offset + 2, 11, buff,1);                         
                    if(KEY_UP == key)
                    {
                        time_unit = !time_unit;
                        time_unit ? sprintf(buff,"ms"):sprintf(buff," s");                        
                        LCD_write_string(row_offset + 2, 11, buff,1);                         

                    }
                    if(KEY_ENTER == key)
                    {
                        LCD_write_string(row_offset + 2, 11, buff,0);
                    }
                    break;                                                                
            }
            if(key == KEY_ENTER) 
            {
                //sprintf(buff,"%d",nghin);
                //LCD_write_string(2, num_offset+6, buff,0);                
                if(num_offset++ == MAX_NUM)
                {
                    //KeyEnter = 1;
                    num_offset = 0;
                    if(row_offset == 0)
                    {
                         stTimeSet.uiTON = nghin*1000+ tram*100 + chuc*10 + dv;
                    }
                    else if(row_offset == 1)
                    {
                         stTimeSet.uiTOFF = nghin*1000+ tram*100 + chuc*10 + dv;
                    }
                    else if(row_offset == 3)
                    {
                        KeyEnter = 3;
                    }
                    row_offset++;                    
                 }
                 if(num_offset != UNIT)
                 {
                    LCD_write_string(2 + row_offset, num_offset + 6, buff,1);   
                 }
                 
               
            }
        }
        else if(KeyEnter == 3)
        {
            static u8 SettingAccept = FALSE;
            switch(key)
            {
                case KEY_UP :
                        //LCD print OK
                    LCD_write_string(4, 6, "OK",1);                   
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
                        // Send message queue Time setting to RELAY task
                    }
                    //Send offset
                    KeyEnter = 0;
                    break;
            }
        } 
        //vTaskDelay(100);   
    }
}

/*RELAY task*/
static void vRelayTask(void *pvParameters)
{
    static TimerSetting_t stTimeset;
    int rl_scan_idx;
    while(TRUE)
    {
        xQueueReceive( qTLTimeOnOff, &stTimeset, ( TickType_t ) 5 );
        for(rl_scan_idx = 0;rl_scan_idx < RL_NUMBER_MAX;rl_scan_idx++)
        {
            GPIO_WriteBit(RELAY_OUTPUT_MAPPING[rl_scan_idx].port,RELAY_OUTPUT_MAPPING[rl_scan_idx].pin, Bit_SET);
            vTaskDelay(stTimeset.uiTON);
            GPIO_WriteBit(RELAY_OUTPUT_MAPPING[rl_scan_idx].port,RELAY_OUTPUT_MAPPING[rl_scan_idx].pin, Bit_RESET);
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

char *Menu[] = 
{
    "TON",
    "TOFF",
    "OK"
};
void DisplayCursorStr(int row)
{
    int i;
    for (i = 0; i < 3; i++)
    {
        if(i == row) 
        {
            LCD_write_string(i+2,0,Menu[i],1);
        }else
        {
            LCD_write_string(i+2,0,Menu[i],0);    
        }
        
    }
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

    /*Initial for led and BL*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*Initial for PWKEY*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // vParTestInitialise();
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);    
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
