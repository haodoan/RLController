#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_lib.h"

/* Demo application includes. */
#include "serial.h"

/* Demo application includes. */
#include "sim908.h"

extern uart_rtos_handle_t uart2_handle;
SemaphoreHandle_t xMutex;
/*Get response from SIMCOM after send AT command*/
uint8_t GetResponse(char *buff_receive, uint32_t timeout)
{
    uint8_t count_char = 0;
    TickType_t xtime;
    signed char SIM_RxChar;
    char cPassMessage[MAX_LENGH_STR];

    xtime = xTaskGetTickCount();

    do
    {
        if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar, 20))
        {
            cPassMessage[count_char++] = SIM_RxChar;
        }
        else
        {
            if((SIM_RxChar == 0xA) && (cPassMessage[count_char -2] == 0xD) && (count_char > 4))
            {
                break;
            }
        }
    } while ((xTaskGetTickCount() - xtime < timeout )&&(count_char < MAX_LENGH_STR));

    if (count_char == MAX_LENGH_STR)
    {
        return pdFALSE;
    }
    cPassMessage[count_char] = '\0';
    strcpy(buff_receive, cPassMessage);

    return pdTRUE;
}

int8_t SendATcommand2(char *ATcommand,char *expected_answer,char *expected_answer2, unsigned int timeout)
{
    char buffer_response[MAX_LENGH_STR];
    signed char SIM_RxChar;
    uint32_t error = pdTRUE;

    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));
        printf("%s\r", ATcommand); // Send the AT command
        if (pdFALSE == GetResponse(buffer_response, timeout))
        {
            error = pdFALSE;
        }
        else
        {
            if (strstr(buffer_response, expected_answer))
            {
                GetResponse(buffer_response, timeout);
                if (strstr(buffer_response, expected_answer2) == NULL)
                {
                    error = pdFALSE;
                }
            }        
        }
        xSemaphoreGive( xMutex );        
    }
       
    return error;
}

int8_t SendATcommand(char *ATcommand, char *expected_answer,unsigned int timeout)
{
    uint8_t count_char = 0;
    uint8_t error = pdTRUE;
    signed char SIM_RxChar;
    char cPassMessage[MAX_LENGH_STR];

    memset(cPassMessage,'\0',MAX_LENGH_STR);
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,200));
        printf("%s\r", ATcommand); // Send the AT command
        do
        {
            if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar,timeout))
            {
                cPassMessage[count_char++] = SIM_RxChar;
            }
            else
            {
                error = pdFAIL;
                break;
            }
            if(strstr(cPassMessage,expected_answer))
            {
                break;
            }
        } while (count_char < MAX_LENGH_STR);

        if ((count_char == MAX_LENGH_STR) || (error == pdFAIL))
        {
            xSemaphoreGive( xMutex );
            return pdFALSE;
        }
        cPassMessage[count_char] = '\0';
        xSemaphoreGive( xMutex );
    }
    return pdTRUE;
}

uint8_t GPS_PWR()
{
   // Power up GPS
     SendATcommand("AT+CGPSPWR=1", "OK", 2000);
    // Reset GPS Hot mode
     SendATcommand("AT+CGPSRST=1", "OK", 2000);
     return pdTRUE;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : start_GPS
 * Description   : Starting GPS for modul SIM908
 * This function
 *
 *END**************************************************************************/
BaseType_t Wait_GPS_Fix(void)
{
    // waits for fix GPS
    if(pdTRUE == SendATcommand("AT+CGPSSTATUS?", "3D Fix", 2000))
    {
        return pdTRUE;
    }
    else
    {
        return(SendATcommand("AT+CGPSSTATUS?", "2D Fix", 2000));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : get_GPS
 * Description   : get_GPS GPRS for modul SIM908
 * This function use config get parameter GPS
 *
 *END**************************************************************************/
uint8_t get_GPS(GPS_INFO *vGPSinfo)
{
    char buffer_response[MAX_LENGH_STR];
    uint32_t error;
    signed char SIM_RxChar;
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    { 
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));
        // First get the NMEA string
        printf("AT+CGPSINF=0\r");
        if(pdTRUE == GetResponse(buffer_response,2000))
        {
            strtok(buffer_response, ",");
            strcpy(vGPSinfo->longtitude,strtok(NULL, ",")); // Gets longitude
            strcpy(vGPSinfo->latitude,strtok(NULL, ",")); // Gets latitude
            //strcpy(altitude,strtok(NULL, ".")); // Gets altitude
            strtok(NULL, ".");
            strtok(NULL, ",");
            strcpy(vGPSinfo->date,strtok(NULL, ".")); // Gets date
            strtok(NULL, ",");
            error = pdTRUE ;
        }
        else {error = pdFAIL;}
        
        xSemaphoreGive( xMutex );
    }
    return error;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : Config_GPRS_SIM908
 * Description   : Config GPRS for modul SIM908
 * This function use config gprs to sim908
 *
 *END**************************************************************************/
void Config_GPRS_SIM908(void)
{
    //    SendATcommand("AT+CIPCSGP=1,\"v-internet\",\"\",\"\"","OK", 2000); // For Viettel Network
    // VinaPhone
    SendATcommand("AT+CIPCSGP=1,\"3m-world\",\"mms\",\"mms\"", "OK", 2000); // For Vina Network
}

/*FUNCTION**********************************************************************
 *
 * Function Name : Tcp_Connect
 * Description   :
 * This function
 *AT+CIPSTART="TCP","42.115.190.28","8888"
 *END**************************************************************************/
TCP_STATUS TCP_Connect(char *IP_address, char *Port,uint32_t timeout)
{
    char command[50];

    memset(command, '\0', 50);
    SendATcommand("AT+CIPSHUT", "SHUT OK", 5000);
    sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", IP_address, Port);
    if (pdTRUE == SendATcommand2(command,"OK","CONNECT OK", timeout))
    {
        return TCP_CONNECT_SUCCESS;
    }
    return TCP_CONNECT_FAIL;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : TCP_Send
 * Description   : `
 * This function
 *
 *END**************************************************************************/
TCP_STATUS TCP_Send(char *data_string)
{
    //char data_ctrl_z[120];
    char *data_ctrl_z;
    TCP_STATUS status;

    data_ctrl_z = pvPortMalloc(strlen(data_string) + 2);//malloc(strlen(data_string) + 2);

    if(data_ctrl_z == NULL) {return TCP_FAIL_MEM;}

    // memset(data_ctrl_z , '\0',120);
    if (pdTRUE == SendATcommand("AT+CIPSEND", ">", 10000))
    {
        sprintf(data_ctrl_z, "%s%c", data_string, 26);
        if (pdFALSE == SendATcommand(data_ctrl_z, "SEND OK", 20000))
        {
            status = TCP_SEND_TIMEOUT;
        }
        else
        {
            status = TCP_SEND_SUCCESS;
        }
    }
    else
    {
        status = TCP_SEND_FAIL;
    }
    vPortFree(data_ctrl_z);

    return status;
}

TCP_STATUS TCP_GetStatus(void)
{

    if (pdTRUE == SendATcommand("AT+CIPSTATUS", "CONNECT OK", 2000))
    {
        return TCP_CONNECT_SUCCESS;
    }
    else
    {
        return TCP_CONNECT_FAIL;
    }
}
/*FUNCTION**********************************************************************
 *
 * Function Name : TCP_Close
 * Description   :
 * This function
 *
 *END**************************************************************************/
TCP_STATUS TCP_Close(void)
{
    // Closes the socket
    if (pdTRUE == SendATcommand("AT+CIPCLOSE", "CLOSE OK", 10000))
    {
        return TCP_SUCCESS;
    }
    return TCP_FAIL;
}

uint8_t GetAccount()
{
    char buffer_acc[160] , *ptr_buff;
    uint16_t cnt = 0;
    char SIM_RxChar;

    ptr_buff = buffer_acc;

    if (SendATcommand("AT+CUSD=1,\"*101#\"", "OK", 2000))
    {
        do {
            if (pdFALSE != xSerialGetChar(&uart2_handle, (signed char*)&SIM_RxChar, 0xffff))
            {
                *(ptr_buff++) = SIM_RxChar;
                cnt++;
            }
            if((*(ptr_buff - 2)==0xD) && (SIM_RxChar ==0xA) && (cnt > 2))
            {
                break;
            }
        }while(1) ;
    }
    *ptr_buff = '\0';
    return pdTRUE;
}
void Sim908_setup(void)
{

    xMutex = xSemaphoreCreateMutex();
    if( xMutex == NULL )
    {
        while(1);
    }
    Sim908_power_on(); // Power up Sim908 module
    //GetIMEI(imei);
    //GetAccount();
    /*****Config Sim908 Module *****************************/
    SendATcommand("AT+CFUN=1", "OK", 2000);       // off echo
    SendATcommand("AT+CIPSHUT", "SHUT OK", 3000); // disconect gprs
    SendATcommand("AT+CSCLK=1", "OK", 2000);      // sleep mode
    SendATcommand("AT+CMGF=1", "OK", 2000);
    // GPIO_WriteLow(DTR_GPIO_PORT, (GPIO_Pin_TypeDef)DTR_GPIO_PINS); //wake up
    // Power up GPS
    //SendATcommand("AT+CGPSPWR=1", "OK", 2000); // power up gps
    // Reset GPS Cold mde
    //SendATcommand("AT+CGPSRST=1", "OK", 2000);
    SendATcommand("AT+CREG=2", "OK", 2000);
    /************End Config Sim908 Module *****************************/
    // delay(1000);
    Config_GPRS_SIM908();
    while (SendATcommand("AT+CREG?", "+CREG: 2,1", 2000) == pdFALSE);
    // Configure DNS server address
    SendATcommand("AT+CGATT", "OK", 2000);
    // delay(1000);
    SendATcommand("AT+CSTT=\"3m-world\",\"mms\",\"mms\"", "OK", 2000);
    SendATcommand("AT+CIICR", "OK", 8000);
    // delay(5000);
    SendATcommand("AT+CIPSTATUS", "OK", 3000);
    // delay(2000);
    SendATcommand("AT+CIFSR", "OK", 3000);
    // delay(4000);
    SendATcommand("AT+CDNSCFG=\"8.8.8.8\",\"4.4.4.4\"", "OK", 2000);
    // delay(2000);
    SendATcommand("AT&W", "OK", 2000);
    /*Config for first time*/
    // Config_GPRS_SIM908();
}

void Sim908_power_on(void)
{

    if (pdFALSE == SendATcommand("AT", "OK", 2000))
    { // power on pulse
        SIM908_PWRON;
        delay_ms(3000);
        SIM908_PWROFF;
        // Wake up
        // waits for an answer from the module
        SendATcommand("ATE0", "OK", 2000);
        while(pdFALSE == SendATcommand("AT", "OK", 2000));
    }
    //SendATcommand("AT", "OK", 2000);
}

void GetCmdDataSIM(char *str , char DATA_AT[5][10])
{
    char * pch;
    int i=0;
    pch = strtok (str,":");

    while (pch != NULL)
    {
        pch = strtok(NULL,",\"\r\n");
        strcpy(DATA_AT[i++],pch);
        if(i == 5) break;
    }
}

/*Get Cell ID*/
void GetCellid(GPS_INFO  *info_cellid )
{
    char buff[32];
    char DATA_AT[5][10] ;
    signed char SIM_RxChar;
    
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        memset(DATA_AT , '\0' , sizeof(DATA_AT));
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));        
        printf("AT+CREG?\r");
        if(GetResponse(buff, 2000))
        {
            GetCmdDataSIM(buff ,DATA_AT);
            strcpy(info_cellid->LAC,strtok (DATA_AT[2],"\""));
            strcpy(info_cellid->CELLID ,strtok (DATA_AT[3],"\""));
            strcpy(info_cellid->latitude ,"0.00");
            strcpy(info_cellid->longtitude ,"0.00");
            strcpy(info_cellid->date ,"0");
        }
        xSemaphoreGive( xMutex );
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : GetIMEI
 * Description   : GetIMEI of Sim module
 * This function use to get id IMEI of Sim module
 *
 *END**************************************************************************/
uint8_t GetIMEI(char * imei)
{
    char buff[32];
    uint32_t error;
    signed char SIM_RxChar;
    
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));
        printf("AT+GSN\r");
        if(GetResponse(buff, 2000))
        {
            strncpy(imei,strstr(buff,"\r\n") + 2,15);
            *(imei+15) = 0;
            error =  pdTRUE;
        }
        else
        {
            error =  pdFALSE;
        }
        xSemaphoreGive( xMutex );        
    }
    return error;
}
