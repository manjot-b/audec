#ifndef HOSTIO_H
#define HOSTIO_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

/**
 * Holds the data from the "info" stage during the initialization protocol.
 */
typedef struct {
	uint32_t sampleRate;
	uint8_t bitDepth;
	uint8_t channels;
	uint16_t dataLength;
} InfoPacket;

void hostIOSetup();
void hostIOTask(void* args);

#endif
