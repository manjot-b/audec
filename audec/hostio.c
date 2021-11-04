/**
 * Module to communiton with the host via UART.
 * 
 * Reads in data and notifies the decoder when the input buffer
 * is full.
 */

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hostio.h"

static void init_protocol(const usart_ctx* usart);
static void send_str(const char* str, const usart_ctx* usart);
static void send_data(const char* data, uint32_t size, const usart_ctx* usart);
static void send_char(char data, const usart_ctx* usart);
static bool recv_str(char* buf, unsigned int size, const usart_ctx* usart, TickType_t timeout);

#define IN_BUF_SIZE 256
static char in_buf[IN_BUF_SIZE];

void hostio_setup(const usart_ctx* usart) {
	rcc_periph_clock_enable(usart->rcc_gpio);
	rcc_periph_clock_enable(usart->rcc_usart);

	gpio_set_mode(usart->gpio,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		usart->tx | usart->rts);
	gpio_set_mode(usart->gpio,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN,
		usart->rx | usart->cts);

	usart_disable(usart->number);
	usart_set_baudrate(usart->number, 115200);
	usart_set_databits(usart->number, 8);
	usart_set_stopbits(usart->number, 1);
	usart_set_mode(usart->number, USART_MODE_TX_RX);
	usart_set_parity(usart->number, USART_PARITY_NONE);
	usart_set_flow_control(usart->number, USART_FLOWCONTROL_RTS_CTS);
	usart_enable(usart->number);
}

static void send_str(const char* str, const usart_ctx* usart) {
	for(; *str; ++str) {
		send_char(*str, usart);
	}
}

static void send_data(const char* data, uint32_t size, const usart_ctx* usart) {
	for(uint32_t i = 0; i < size; i++) {
		send_char(data[i], usart);
	}
}

static void send_char(char data, const usart_ctx* usart) {
	while ( !usart_get_flag(usart->number, USART_SR_TXE) ) {
		taskYIELD();
	}

	usart_send(usart->number, data);
}

/**
 * Receives a string in the specified timeout period or until '\r\n' is received.
 *
 * @param buf[out] The buffer to store the string into. @note '\r\n' will be stripped.
 * @param size The size of the buffer.
 * @param usart The USART context to receive data from.
 * @param timeout The amount of time to wait before returning. If set to 0 then the function
 * waits indefinetly.
 *
 * @return Whether a valid string was received within the timeout period.
 */
static bool recv_str(char* buf, unsigned int size, const usart_ctx* usart, TickType_t timeout) {
	TickType_t ticks_old = xTaskGetTickCount();
	bool timed_out = false;
	bool received_crlf = false;

	unsigned int i = 0;

	while (i < size && !received_crlf) {
		while ( usart_get_flag(usart->number, USART_SR_RXNE) == 0 ) {
			taskYIELD();

			timed_out = timeout > 0 && xTaskGetTickCount() - ticks_old > timeout;
			if (timed_out) {
				break;
			}
		}

		buf[i] = usart_recv(usart->number);

		if (i > 0) {
			received_crlf = buf[i-1] == '\r' && buf[i] == '\n';
			if (received_crlf) {
				buf[i-1] = '\0';
			}
		}

		i++;
	}

	return received_crlf;
}

static void init_protocol(const usart_ctx* usart) {
	bool valid_response = false;
	char recv[16];

	char buf_sz[] = {IN_BUF_SIZE >> 8, IN_BUF_SIZE & 0xFF};

	while (!valid_response) {
		
		send_str("bfsz\r\n", usart);
		send_data(buf_sz, 2, usart);

		if (recv_str(recv, 16, usart, 2000)) {
			valid_response = strcmp(recv, "info") == 0;

			if (valid_response) {
				send_str("good\r\n", usart);
			} else {
				send_str("bad\r\n", usart);
			}
		}
	}
}

void hostio_task(void* args) {
	usart_ctx* usart = (usart_ctx*)args;

	for (;;) {
		init_protocol(usart);
		
		while(1);	
	}
}

