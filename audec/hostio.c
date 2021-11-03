/**
 * Module to communiton with the host via UART.
 * 
 * Reads in data and notifies the decoder when the input buffer
 * is full.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hostio.h"

static void request_data(const usart_ctx* usart);
static void send_str(const char* str, const usart_ctx* usart);
static void send_char(char data, const usart_ctx* usart);

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

static void send_char(char data, const usart_ctx* usart) {
	while ( !usart_get_flag(usart->number, USART_SR_TXE) ) {
		taskYIELD();
	}

	usart_send(usart->number, data);
}

static void request_data(const usart_ctx* usart) {
	bool response_recv = false;

	while (!response_recv) {
		send_str("bfsz\r\n", usart);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}

}

void hostio_task(void* args) {
	usart_ctx* usart = (usart_ctx*)args;

	for (;;) {
		request_data(usart);	
	}
}

