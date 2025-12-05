/*
 * hardware.c
 *
 *  Created on: Jul 9, 2024
 *      Author: smkim
 */
#include "main.h"

uint16_t usUart3RxIdleMs = 0;
uint16_t usUart3TxIdleMs = 0;
uint32_t unTimer1MsTicks = 0;
uint32_t unSWCount1MsTicks = 0;
uint8_t ucSW_Input[4] = {0,};
uint8_t ucDIP_SW = 0;

void Timer_Ms_Ticks_Proc(void)
{
	if(usUart3RxIdleMs) usUart3RxIdleMs--;
	if(usUart3TxIdleMs) usUart3TxIdleMs--;
	unTimer1MsTicks++;
	unSWCount1MsTicks++;
}

uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen)
{
	hqspi.Instance->DLR=datalen-1;
    if(HAL_QSPI_Receive(&hqspi,buf,5000)==HAL_OK) return 0;
    else return 1;
}

uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen)
{
    hqspi.Instance->DLR=datalen-1;
    if(HAL_QSPI_Transmit(&hqspi,buf,5000)==HAL_OK) return 0;
    else return 1;
}

uint8_t GPIO_DIP_SW_Input(void)
{
	uint8_t sw_in=0;
	if(unSWCount1MsTicks>=20)
	{
		unSWCount1MsTicks = 0;
		ucSW_Input[0] <<= 1;
		ucSW_Input[0] |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
		ucSW_Input[1] <<= 1;
		ucSW_Input[1] |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
		ucSW_Input[2] <<= 1;
		ucSW_Input[2] |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
		ucSW_Input[3] <<= 1;
		ucSW_Input[4] |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);

		if((ucSW_Input[0] & 0x1f) == 0) ucDIP_SW |= 0x08;
		else if((ucSW_Input[0] & 0x1f) == 0x1f) ucDIP_SW &= 0x07;
		if((ucSW_Input[1] & 0x1f) == 0) ucDIP_SW |= 0x04;
		else if((ucSW_Input[1] & 0x1f) == 0x1f) ucDIP_SW &= 0x0B;
		if((ucSW_Input[2] & 0x1f) == 0) ucDIP_SW |= 0x02;
		else if((ucSW_Input[2] & 0x1f) == 0x1f) ucDIP_SW &= 0x0D;
		if((ucSW_Input[3] & 0x1f) == 0) ucDIP_SW |= 0x01;
		else if((ucSW_Input[3] & 0x1f) == 0x1f) ucDIP_SW &= 0x0E;
	}
	return ucDIP_SW;
}
