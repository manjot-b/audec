#ifndef HOSTIO_H
#define HOSTIO_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

/**
 * USART Context
 */
typedef struct {
	enum rcc_periph_clken rcc_gpio;
	enum rcc_periph_clken rcc_usart;
	uint32_t number;
	uint32_t gpio;
	uint16_t tx;
	uint16_t rx;
	uint16_t cts;
	uint16_t rts;
} usart_ctx;

/**
 * Holds the data from the "info" stage during the initialization protocol.
 */
typedef struct {
	uint32_t sample_rate;
	uint8_t bit_depth;
	uint8_t channels;
	uint16_t data_length;
} info_packet;

void hostio_setup(const usart_ctx* usart);
void hostio_task(void* args);

#endif
