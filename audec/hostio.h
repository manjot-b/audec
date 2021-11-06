#ifndef HOSTIO_H
#define HOSTIO_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

/**
 * USART Context
 */
typedef struct {
	enum rcc_periph_clken rccGpio;
	enum rcc_periph_clken rccUsart;
	uint32_t number;
	uint32_t gpio;
	uint16_t tx;
	uint16_t rx;
	uint16_t cts;
	uint16_t rts;
} UsartContext;

/**
 * Holds the data from the "info" stage during the initialization protocol.
 */
typedef struct {
	uint32_t sampleRate;
	uint8_t bitDepth;
	uint8_t channels;
	uint16_t dataLength;
} InfoPacket;

void hostIOSetup(const UsartContext* usart);
void hostIOTask(void* args);

#endif
