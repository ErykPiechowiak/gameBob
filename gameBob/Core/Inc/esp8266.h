/*
 * esp8266.h
 *
 *  Created on: Mar 2, 2025
 *      Author: Eryk
 */

#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

#include "stm32f1xx_hal.h"
#include "stddef.h"

#define MAX_RX_BUFFER 512
#define ESP8266_OK 1
#define ESP8266_ERROR 2
#define ESP8266_BUSY 0
#define ESP_UART_NOT_DEFINED 3


typedef enum {
	DEFAULT,
	IP_SEND,
	GET_REQUEST
}CmdType;


extern uint8_t rx_buffer[MAX_RX_BUFFER];
extern uint8_t rx_index;
extern volatile uint8_t tx_completed_flag;
extern volatile uint8_t found_flag;

extern UART_HandleTypeDef *huart_esp;

uint8_t Esp8266_Init(UART_HandleTypeDef *huart);
uint8_t Esp8266_SetCwMode(uint8_t cw_mode);
uint8_t Esp8266_ListAP(char *output);
uint8_t Esp8266_ConnectAP(char *ssid, char *psw);
uint8_t Esp8266_GetIpAddress(char *buffer, size_t size);
uint8_t Esp8266_StartTCPIPConnection(char *server, char *port);
uint8_t Esp8266_SendIpCommand(char *cmd);
uint8_t Esp8266_ChangeBaudRate(int baudrate, int8_t data_bits, int8_t stop_bits, int8_t parity, int8_t flow_control);

void ESP8266_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void ESP8266_UART_RxCpltCallback(UART_HandleTypeDef *huart);


static void reset_flags();
static uint8_t sendAndReceive(char *cmd);
static uint8_t Esp8266_Reset();
#endif /* INC_ESP8266_H_ */
