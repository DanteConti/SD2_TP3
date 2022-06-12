/*==================[inclusions]=============================================*/
// Standard C Included Files
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

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
#include "uart0_drv.h"
#include "ringBuffer.h"


/*==================[macros and definitions]=================================*/
#define LPUART0_BAUD_RATE 115200
#define TP3_UART LPUART0
#define TP3_UART_IRQ LPUART0_IRQn
#define RB_SIZE 21
#define UART_TX_DMA_CHANNEL 0U
#define TX_BUFFER_DMA_SIZE  32

/*==================[internal data declaration]==============================*/
static uint8_t txBuffer_dma[TX_BUFFER_DMA_SIZE];
static lpuart_dma_handle_t lpuartDmaHandle;
static dma_handle_t lpuartTxDmaHandle;
static void *rxRingBufferPtr;
volatile bool txOnGoingDma0, txOnGoingUart0;

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void LPUART_UserCallback(LPUART_Type *base, lpuart_dma_handle_t *handle, status_t status, void *userData){

	txOnGoingDma0 = false;
}

void LPUART0_IRQHandler(void){
    if( (kLPUART_RxDataRegFullFlag 			  & LPUART_GetStatusFlags(TP3_UART)) &&
    	(kLPUART_RxDataRegFullInterruptEnable & LPUART_GetEnabledInterrupts(TP3_UART)) ){

    	uint8_t rxByte;
    	rxByte = LPUART_ReadByte(TP3_UART); // limpio bandera ISR
    	ringBuffer_putData(rxRingBufferPtr, rxByte);
    	LPUART_ClearStatusFlags(TP3_UART, kLPUART_RxOverrunFlag);
    }
    if( (kLPUART_TransmissionCompleteFlag 			 & LPUART_GetStatusFlags(TP3_UART)) &&
		(kLPUART_TransmissionCompleteInterruptEnable & LPUART_GetEnabledInterrupts(TP3_UART)) ){

    	LPUART_DisableInterrupts(TP3_UART, kLPUART_TransmissionCompleteInterruptEnable);
    	LPUART_ClearStatusFlags(TP3_UART, kLPUART_TransmissionCompleteFlag);
    	txOnGoingUart0 = false;
    }
}

/*==================[external functions definition]==========================*/

void uart0_drv_init(void){

	lpuart_config_t config;

    CLOCK_SetLpuart0Clock(0x1U);

    CLOCK_EnableClock(kCLOCK_PortA);

    /* PORTA1 (pin 35) is configured as LPUART0_RX */
    PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);

    /* PORTA2 (pin 36) is configured as LPUART0_TX */
    PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);

    /*
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = LPUART0_BAUD_RATE;
    config.parityMode = kLPUART_ParityDisabled;
    config.stopBitCount = kLPUART_OneStopBit;
    config.enableTx = true;
    config.enableRx = true;

    LPUART_Init(TP3_UART, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));

    /* Habilita interrupciones */
    LPUART_EnableInterrupts(TP3_UART, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(TP3_UART_IRQ);

    rxRingBufferPtr = ringBuffer_init(RB_SIZE);

    /* CONFIGURACIÓN DMA (sólo para TX) */
    /* Init DMAMUX */
    DMAMUX_Init(DMAMUX0);

    /* Set channel for LPUART  */
    DMAMUX_SetSource(DMAMUX0, UART_TX_DMA_CHANNEL, kDmaRequestMux0LPUART0Tx);
    DMAMUX_EnableChannel(DMAMUX0, UART_TX_DMA_CHANNEL);

    /* Init the DMA module */
    DMA_Init(DMA0);
    DMA_CreateHandle(&lpuartTxDmaHandle, DMA0, UART_TX_DMA_CHANNEL);

    /* Create LPUART DMA handle. */
    LPUART_TransferCreateHandleDMA(
            TP3_UART,
            &lpuartDmaHandle,
            LPUART_UserCallback,
            NULL,
            &lpuartTxDmaHandle,
            NULL);
}

int32_t uart0_drv_envDatos(uint8_t *pBuf, int32_t size){
	if(txOnGoingDma0 || txOnGoingUart0){
		return 0;
	}else{
		lpuart_transfer_t xfer;

		if(size > TX_BUFFER_DMA_SIZE){
			size = TX_BUFFER_DMA_SIZE;
		}
		memcpy(txBuffer_dma, pBuf, size);
		xfer.data = txBuffer_dma;
		xfer.dataSize = size;
		txOnGoingDma0 = true;
		LPUART_TransferSendDMA(TP3_UART, &lpuartDmaHandle, &xfer);
		LPUART_EnableInterrupts(TP3_UART, kLPUART_TransmissionCompleteInterruptEnable);
		return(size);
	}
}

/** \brief recibe datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde guardar los datos
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes recibidos
 **/
int32_t uart0_drv_recDatos(uint8_t *pBuf, int32_t size){
    int32_t ret = 0;

    /* entra sección de código crítico */
    DisableIRQ(TP3_UART_IRQ);

    while (!ringBuffer_isEmpty(rxRingBufferPtr) && ret < size){
        ringBuffer_getData(rxRingBufferPtr, &pBuf[ret]);
        ret++;
    }

    /* sale de sección de código crítico */
    EnableIRQ(TP3_UART_IRQ);

    return ret;
}


/*==================[end of file]============================================*/
