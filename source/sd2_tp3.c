#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "SD2_board_KL43.h"
#include "MEF.h"
#include "ringBuffer.h"
#include "mma8451.h"
#include "SD2_I2C.h"
/* TODO: insert other definitions and declarations here. */

#define CMD_BUFFER_SIZE 21

void *cmdBuffer;

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

    /* Init USB0 */
    uart0_drv_init();

    /* Init mef Rx */
    //MefRxInit();
    cmdBuffer = ringBufferComando_init(CMD_BUFFER_SIZE);

    while(1) {
    	MefRxTick(cmdBuffer);
    	MefProcesamientoTick(cmdBuffer);
    }
    return 0 ;
}
