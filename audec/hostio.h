#ifndef HOSTIO_H
#define HOSTIO_H

#include "hostio.h"

extern TaskHandle_t hostIOHandle;
/**
 * Holds the data from the "info" stage during the initialization protocol.
 */
typedef struct {
	uint32_t sampleRate;
	uint8_t bitDepth;
	uint8_t channels;
	uint16_t dataLength;
} InfoPacket;

/**
 * Setup USART and DMA registers for communication with the host.
 */
void hostIOSetup(void);

/**
 * The task to communicate with the host.
 *
 * Performs the initial handshake and if it suceeds then setups DMA.
 * Waits for the DMA transfer to be complete before notifying the decodeder that
 * @c inputBuf is ready to be decoded.
 */
void hostIOTask(void* args);

#endif
