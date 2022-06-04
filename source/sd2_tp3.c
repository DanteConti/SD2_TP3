#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "SD2_board_KL43.h"
#include "MEF.h"
/* TODO: insert other definitions and declarations here. */


int main(void) {

    /* Init board hardware. */
    //BOARD_InitBootPins();
    BOARD_InitBootClocks();
    //BOARD_InitBootPeripherals();

    /* Init USB0 */
    uart0_drv_init();

    /* Init mef Rx */
    MEF_RX_init();
    Comando *comando;

    while(1) {
    	comando = MEF_RX_tick(comando);
    	if(comando != NULL && comando->pedido == LECTURA){
    		uint8_t msg[] = "ok";
    		uart0_drv_envDatos(msg, sizeof(msg)/sizeof(msg[0]));
    	}
    }
    return 0 ;
}
