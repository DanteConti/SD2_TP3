/* Librerias generales */
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"

/* Librerias del proyecto */
#include "SD2_board_KL43.h"
#include "MEF.h"
#include "uart0_drv.h"
#include "uart2_drv.h"
#include "ringBuffer.h"
#include "mma8451.h"
#include "SD2_I2C.h"

/* Macro para elegir entre LPUart0 o Uart2
 * Comentar para usar Uart2
 * Descomentar para usar LPUart0
 */
//#define USE_UART0

#define CMD_BUFFER_SIZE 21

/* Buffer de comandos recibidos */
void *cmdBuffer;

/* Punteros a funciones */
static bool uartReadByte(uint8_t*);
static int32_t uartSend(uint8_t*, int32_t);
static void uartInit(void);

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootClocks();

    /* Init pines GPIO */
	board_init();

    /* Init I2C */
    SD2_I2C_init();

    /* Init acelerometro */
    mma8451_init_continuous();
    mma8451_setDataRate(DR_12p5hz);

    /* Init UART */
    uartInit();

    /* Init mef Rx */
    MefRxInit(uartReadByte);

    /* Init mef Procesamiento */
    MefProcesamientoInit(uartSend);

    /* Init buffer comandos recibidos */
    cmdBuffer = ringBufferComando_init(CMD_BUFFER_SIZE);

    while(1) {
    	MefRxTick(cmdBuffer);
    	MefProcesamientoTick(cmdBuffer);
    }
    return 0 ;
}

#ifdef USE_UART0

bool uartReadByte(uint8_t *strByte){
	return (uart0_drv_recDatos(strByte, 1));
}

int32_t uartSend(uint8_t *sendBuffer, int32_t size){
	return (uart0_drv_envDatos(sendBuffer, size));
}

void uartInit(){
	uart0_drv_init();
}

/* IS_UART0 */
#else

bool uartReadByte(uint8_t *strByte){
	return (uart2_drv_recDatos(strByte, 1));
}

int32_t uartSend(uint8_t *sendBuffer, int32_t size){
	return (uart2_drv_envDatos(sendBuffer, size));
}

void uartInit(){
	uart2_drv_init();
}

#endif
