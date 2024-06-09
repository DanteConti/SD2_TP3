/*
 * MEF.h
 *
 *  Created on: 25 may. 2022
 *      Author: Franco
 */

#ifndef MEF_H_
#define MEF_H_

#include "SD2_board_KL43.h"

#define STX 0x3A // ":"
#define ETX 0x0A // LF

/* ================== [Punteros a funciones] ================== */

typedef bool (*uartReadByte_t)(char*);
typedef int32_t (*uartSend_t)(char*, int32_t);

/* ================== [Enumeraciones y estructuras públicas] ================== */
// Mef Rx
typedef enum{
	LED_T = 0,
	SW_T,
	ACC_T,
	LSNS_T,
	ERROR_T
}TipoPeriferico;

typedef enum {
	LED_ID_ROJO = 0,
	LED_ID_VERDE,
	SW_ID_1,
	SW_ID_3,
	ACC,
	LSNS,
	ID_ERROR
}IdPeriferico;

typedef enum {
	ENCENDER = 0,
	APAGAR,
	TOGGLE,
	LECTURA,
	TIPO_COMANDO_ERROR
}TipoComando;

typedef struct __attribute__((__packed__)){
	IdPeriferico periferico;
	TipoComando pedido;
}Comando;

// Mef Procesamiento

/* ================== [Funciones públicas] ================== */
// Mef Rx
void MefRxInit(uartReadByte_t);
void MefRxTick(void*);

// Mef Procesamiento
void MefProcesamientoInit(uartSend_t);
void MefProcesamientoTick(void*);

#endif /* MEF_H_ */
