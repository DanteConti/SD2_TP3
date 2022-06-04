/*
 * MEF.c
 *
 *  Created on: 30 may. 2022
 *      Author: Franco
 */

#include "MEF.h"
#include "string.h"
#include "ringBuffer.h"
#include "uart0_drv.h"

/* ================== [Enumeraciones y estructuras para determinar el comando recibido] ================== */


/* ==================[Enumeracion para MEF de Recepcion de datos por UART] ================== */
typedef enum{
	MefRx_EsperandoSTX = 0,
	MefRx_LeyendoID,
	MefRx_LeyendoPerifericoMSB,
	MefRx_LeyendoPerifericoLSB,
	MefRx_LeyendoAccion,
	MefRx_EsperandoETX
}MEF_Rx_estados;


/* ==================[variables internas] ================== */
//void *cmdBuffer;

/* ================== [funciones internas para MEF RX] ================== */
/*
 * determina sobre que periferico se quiere
 * llevar a cabo la accion, dado por la
 * trama recibida por UART.
 */
IdPeriferico definePeriferico(TipoPeriferico tipoPeriferico, uint8_t tmpRxByte){
	IdPeriferico retVal;

	switch(tipoPeriferico){
		case LED_T:
			if(tmpRxByte == 0x31){ // ID recibido 01
				retVal = LED_ID_ROJO;
			}else if(tmpRxByte == 0x32){ // Id recibido 02
				retVal = LED_ID_VERDE;
			}else{
				retVal = ERROR_T;
			}
			break;
		case SW_T:
			if(tmpRxByte == 0x31){ // ID recibido 11
				retVal = SW_ID_1;
			}else if(tmpRxByte == 0x33){ // ID recibido 13
				retVal = SW_ID_3;
			}else{
				retVal = ERROR_T;
			}
			break;
		case ACC_T:
			if(tmpRxByte == 0x30){ // ID recibido 20
				retVal = ACC;
			}else{
				retVal = ERROR_T;
			}
			break;
		case ERROR_T:
			retVal = ERROR_T;
			break;
	}
	return retVal;
}

bool MEF_readByte(uint8_t *retPtr){
	return (uart0_drv_recDatos(retPtr, 1) == 1);
}

void mefRx_guardarComando(Comando *buffer, uint8_t bufferSize, Comando data){
	static uint8_t bufferIndex = 0;

	buffer[bufferIndex] = data;
	(bufferIndex < bufferSize-1) ? (bufferIndex++) : (bufferIndex = 0);
}


/* ================== [funciones publicas para MEF RX] ================== */
void MEF_RX_init(){
	//cmdBuffer = ringBuffer_init(RB_SIZE);
}

#define TEST_RX_SIZE 18
int8_t testRx[TEST_RX_SIZE] = {STX, ':', 0x30, 0x31, 0x31, ETX, STX, 0x31, 0x30, 0x31, 0x33, ETX, STX, 0x31, 0x30, 0x32, 0x30, ETX}; // ":1001E'LF'"

Comando *MEF_RX_tick(Comando *bufferComando){
	static MEF_Rx_estados estado = MefRx_EsperandoSTX;
	static bool start, byteAvailable;
	static TipoPeriferico tmpPeriferico = ERROR_T;
	static TipoComando tmpComando = TIPO_COMANDO_ERROR;
	static Comando comando;
	uint8_t rxChar;

	byteAvailable = uart0_drv_recDatos(&rxChar,1);

	switch(estado){
		case MefRx_EsperandoSTX:
			if(byteAvailable){
				if(rxChar == STX){ // ':'
					estado = MefRx_LeyendoID;
					start = true;
				}
			}
			break;
		case MefRx_LeyendoID:
			if(byteAvailable){
				if(rxChar == STX){
					start = true;
					break;
				}
				if(start){
					(rxChar == 0x31) ? (start = false) : (estado = MefRx_EsperandoSTX);
				}else{
					if(rxChar == 0x30){
						estado = MefRx_LeyendoPerifericoMSB;
					}else{
						estado = MefRx_EsperandoSTX;
					}
				}
			}
			break;
		case MefRx_LeyendoPerifericoMSB:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				if(rxChar == 0x30){
					tmpPeriferico = LED_T;
				}else if(rxChar == 0x31){
					tmpPeriferico = SW_T;
				}else if(rxChar == 0x32){
					tmpPeriferico = ACC_T;
				}else{
					tmpPeriferico = ERROR_T;
				}
				if(tmpPeriferico == ERROR_T){
					estado = MefRx_EsperandoSTX;
				}else{
					estado = MefRx_LeyendoPerifericoLSB;
				}
			}
			break;
		case MefRx_LeyendoPerifericoLSB:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				comando.periferico = definePeriferico(tmpPeriferico, rxChar);
				if(comando.periferico == ID_ERROR){
					estado = MefRx_EsperandoSTX;
				}else if(comando.periferico == LED_ID_ROJO || comando.periferico == LED_ID_VERDE){
					estado = MefRx_LeyendoAccion;
				}else{
					estado = MefRx_EsperandoETX;
					comando.pedido = LECTURA;
				}
			}
			break;
		case MefRx_LeyendoAccion:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				if(rxChar == 0x45){ // accion recibida E
					tmpComando = ENCENDER;
				}else if(rxChar == 0x41){ // accion recibida A
					tmpComando = APAGAR;
				}else if(rxChar == 0x54){ // accion recibida T
					tmpComando = TOGGLE;
				}else{
					tmpComando = TIPO_COMANDO_ERROR;
				}
				if(tmpComando == TIPO_COMANDO_ERROR){
					estado = MefRx_EsperandoSTX;
				}else{
					estado = MefRx_EsperandoETX;
					comando.pedido = tmpComando;
				}
			}
			break;
		case MefRx_EsperandoETX:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				if(rxChar == ETX){
					estado = MefRx_EsperandoSTX;
					return &comando;
				}else{
					estado = MefRx_EsperandoSTX;
				}
			}
			break;
	}
	return NULL;
}
