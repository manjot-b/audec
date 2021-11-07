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

void hostIOSetup(void);
void hostIOTask(void* args);

#endif
