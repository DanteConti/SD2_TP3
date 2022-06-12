/*
 * uart2_drv.c
 *
 *  Created on: 12 jun. 2022
 *      Author: dante
 */


// Project Included Files
#include "SD2_board_KL43.h"
#include "fsl_lpuart.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_lpuart_dma.h"
#include "fsl_uart_dma.h"
#include "fsl_dmamux.h"
#include "board.h"
#include "MKL43Z4.h"
#include "pin_mux.h"
#include "ringBuffer.h"
#include "uart2_drv.h"


#define UART2_BAUD_RATE 115200
#define UART_TX_DMA_CHANNEL 0U
#define RB_SIZE 21
#define TX_BUFFER_DMA_SIZE  32
#define PE_RE_MASK ( (1<<7) | (1<<5) )

static uint8_t txBuffer_dma[TX_BUFFER_DMA_SIZE];
static void *rxRingBufferPtr;
uart_dma_handle_t uartDmaHandle;
dma_handle_t uartTxDmaHandle;
volatile bool txOnGoingDma2, txOnGoingUart2;

static void UART2_UserCallback(UART_Type *base, uart_dma_handle_t *handle, status_t status, void *userData){

	txOnGoingDma2 = false;
	txOnGoingUart2 = true;
}

void UART2_FLEXIO_IRQHandler(void){
    if( (kUART_RxDataRegFullFlag 			& UART_GetStatusFlags(UART2)) &&
    	(kUART_RxDataRegFullInterruptEnable & UART_GetEnabledInterrupts(UART2)) ){

    	uint8_t rxByte;
    	rxByte = UART_ReadByte(UART2); // limpio bandera ISR
    	ringBuffer_putData(rxRingBufferPtr, rxByte);
    	UART_ClearStatusFlags(UART2, kUART_RxOverrunFlag);
    }
    if( (kUART_TransmissionCompleteFlag 		   & UART_GetStatusFlags(UART2)) &&
		(kUART_TransmissionCompleteInterruptEnable & UART_GetEnabledInterrupts(UART2)) ){

    	UART_DisableInterrupts(UART2, kUART_TransmissionCompleteInterruptEnable);
    	UART_ClearStatusFlags(UART2, kUART_TransmissionCompleteFlag);
    	txOnGoingUart2 = false;
    	/* PE = 0 ; RE = 0 ==> habilita rx */
    	GPIO_PortClear(GPIOC, PE_RE_MASK);
    }
}

void uart2_drv_init(){
	/* Conexiones UART2
	 * UART2_Tx => PTD3 => J1P06
	 * UART2_Rx => PTD2 => J2P04
	 * RE 		=> PTC7 => J1P11
	 * DE		=> PTC5 => J1P15
	 * */

	/* Pin init */
	gpio_pin_config_t deReConfig = {
		.outputLogic = 0,
		.pinDirection = kGPIO_DigitalOutput,
	};

	const port_pin_config_t deRePortConfig = {
		/* Internal pull-up/down resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Slow slew rate is configured */
		.slewRate = kPORT_SlowSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};

	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortD);

	PORT_SetPinConfig(PORTC, 7, &deRePortConfig);
	PORT_SetPinConfig(PORTC, 5, &deRePortConfig);

	GPIO_PinInit(GPIOC, 7, &deReConfig);
	GPIO_PinInit(GPIOC, 5, &deReConfig);

	GPIO_PortClear(GPIOC, PE_RE_MASK);

	PORT_SetPinMux(PORTD, 2, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTD, 3, kPORT_MuxAlt3);

	/* UART2 init */
	uart_config_t uartConfig;

	UART_GetDefaultConfig(&uartConfig);
		uartConfig.baudRate_Bps = UART2_BAUD_RATE;
		uartConfig.enableTx     = true;
		uartConfig.enableRx     = true;

	UART_Init(UART2, &uartConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	UART_EnableInterrupts(UART2, kUART_RxDataRegFullInterruptEnable);
	UART_EnableInterrupts(UART2, kLPUART_TransmissionCompleteInterruptEnable);
	EnableIRQ(UART2_FLEXIO_IRQn);

	/* DMA Init */
	DMAMUX_Init(DMAMUX0);

	/* Set channel for LPUART  */
	DMAMUX_SetSource(DMAMUX0, UART_TX_DMA_CHANNEL, kDmaRequestMux0UART2Tx);
	DMAMUX_EnableChannel(DMAMUX0, UART_TX_DMA_CHANNEL);

	/* Init the DMA module */
	DMA_Init(DMA0);
	DMA_CreateHandle(&uartTxDmaHandle, DMA0, UART_TX_DMA_CHANNEL);

	UART_TransferCreateHandleDMA(
			UART2,
			&uartDmaHandle,
			UART2_UserCallback,
			NULL,
			&uartTxDmaHandle,
			NULL);

	rxRingBufferPtr = ringBuffer_init(RB_SIZE);
}

int32_t uart2_drv_envDatos(uint8_t *pBuf, int32_t size){
	if(txOnGoingDma2 || txOnGoingUart2){
		return 0;
	}else{
		uart_transfer_t xfer;

		if(size > TX_BUFFER_DMA_SIZE){
			size = TX_BUFFER_DMA_SIZE;
		}
		memcpy(txBuffer_dma, pBuf, size);
		xfer.data = txBuffer_dma;
		xfer.dataSize = size;
		txOnGoingDma2 = true;
		/* PE = 1 ; RE = 1 ==> habilita tx */
		GPIO_PortSet(GPIOC, PE_RE_MASK);
		UART_TransferSendDMA(UART2, &uartDmaHandle, &xfer);
		UART_EnableInterrupts(UART2, kUART_TransmissionCompleteInterruptEnable);
		return(size);
	}
}

/** \brief recibe datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde guardar los datos
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes recibidos
 **/
int32_t uart2_drv_recDatos(uint8_t *pBuf, int32_t size){
    int32_t ret = 0;

    /* entra sección de código crítico */
    DisableIRQ(UART2_FLEXIO_IRQn);

    while (!ringBuffer_isEmpty(rxRingBufferPtr) && ret < size){
        ringBuffer_getData(rxRingBufferPtr, &pBuf[ret]);
        ret++;
    }

    /* sale de sección de código crítico */
    EnableIRQ(UART2_FLEXIO_IRQn);

    return ret;
}
