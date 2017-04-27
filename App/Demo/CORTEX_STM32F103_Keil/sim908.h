/* Standard includes. */
#include <stdio.h>

/* Library includes. */
#include "stm32f10x_lib.h"
#include "task.h"

typedef enum _tcp_status
{
    TCP_CONNECT_SUCCESS = 0 ,
    TCP_CONNECT_FAIL ,
    TCP_CONNECT_TIMEOUT,
    TCP_SEND_SUCCESS,
    TCP_SEND_FAIL ,
    TCP_SEND_TIMEOUT ,
    TCP_SUCCESS,
    TCP_FAIL,
    TCP_FAIL_MEM
} TCP_STATUS ;

typedef struct gps_info_t
{
    char IMEI[20];
	char latitude[20];
	char longtitude[20];
	char date[20];
    char LAC[6];
    char CELLID[6];
    uint16_t  MNC;
    uint16_t  MCC;
    uint32_t  FIX;
}GPS_INFO;
#define MAX_LENGH_STR  100
#define SIM908_PWRON   GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET)
#define SIM908_PWROFF  GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET)
#define delay_ms(x)    vTaskDelay(x)

uint8_t GetResponse(char *buff_receive, uint32_t timeout);
int8_t SendATcommand(char *ATcommand, char *expected_answer, unsigned int timeout);
int8_t SendATcommand2(char *ATcommand,char *expected_answer,char *expected_answer2, unsigned int timeout);
uint8_t start_GPS(void);
BaseType_t Wait_GPS_Fix(void);
uint8_t get_GPS(GPS_INFO *vGPSinfo);
void Config_GPRS_SIM908(void);
TCP_STATUS TCP_Connect(char *IP_address, char *Port, unsigned int timeout);
TCP_STATUS TCP_Send(char *data_string)	;
TCP_STATUS TCP_Close(void);
TCP_STATUS TCP_GetStatus(void);
void Sim908_setup(void);
void Sim908_power_on(void);
uint8_t GPS_PWR(void);
uint8_t GetAccount(void);
void GetCellid(GPS_INFO  *info_cellid );
void GetCmdDataSIM(char *str , char DATA_AT[5][10]);
uint8_t GetIMEI(char * imei);
