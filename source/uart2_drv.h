/*
 * uart2_drv.h
 *
 *  Created on: 12 jun. 2022
 *      Author: dante
 */

#ifndef UART2_DRV_H_
#define UART2_DRV_H_

#include "stdint.h"
#include "stdbool.h"

/* ================== [funciones publicas] ================== */

void uart2_drv_init(void);
int32_t uart2_drv_recDatos(uint8_t*, int32_t);
int32_t uart2_drv_envDatos(uint8_t*, int32_t);

#endif /* UART2_DRV_H_ */
