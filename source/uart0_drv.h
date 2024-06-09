#ifndef UART0_DRV_H_
#define UART0_DRV_H_

/*==================[inclusions]=============================================*/
#include "stdint.h"
#include "stdbool.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions definition]==========================*/

void uart0_drv_init(void);
int32_t uart0_drv_recDatos(char*, int32_t);
int32_t uart0_drv_envDatos(char*, int32_t);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* UART0_DRV_H_ */
