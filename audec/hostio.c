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

static UsartContext usart = {
	.rccGpio	= RCC_GPIOB,
	.rccUsart	= RCC_USART3,
	.number		= USART3,
	.gpio		= GPIOB,
	.tx			= GPIO_USART3_TX,
	.rx			= GPIO_USART3_RX,
	.cts		= GPIO13,
	.rts		= GPIO14,
};


static bool initProtocol(InfoPacket* info);
static void sendString(const char* str);
static void sendData(const char* data, uint32_t size);
static void sendChar(char data);
static bool receiveString(char* buf, unsigned int size, TickType_t timeout);
static unsigned int receiveData(char* buf, unsigned int size, TickType_t timeout);

#define IN_BUF_SIZE 256
static char in_buf[IN_BUF_SIZE];

void hostIOSetup() {
	rcc_periph_clock_enable(usart.rccGpio);
	rcc_periph_clock_enable(usart.rccUsart);

	gpio_set_mode(usart.gpio,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		usart.tx | usart.rts);
	gpio_set_mode(usart.gpio,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN,
		usart.rx | usart.cts);

	usart_disable(usart.number);
	usart_set_baudrate(usart.number, 115200);
	usart_set_databits(usart.number, 8);
	usart_set_stopbits(usart.number, 1);
	usart_set_mode(usart.number, USART_MODE_TX_RX);
	usart_set_parity(usart.number, USART_PARITY_NONE);
	usart_set_flow_control(usart.number, USART_FLOWCONTROL_RTS_CTS);
	usart_enable(usart.number);
}

static void sendString(const char* str) {
	for(; *str; ++str) {
		sendChar(*str);
	}
}

static void sendData(const char* data, uint32_t size) {
	for(uint32_t i = 0; i < size; i++) {
		sendChar(data[i]);
	}
}

static void sendChar(char data) {
	while ( !usart_get_flag(usart.number, USART_SR_TXE) ) {
		taskYIELD();
	}

	usart_send(usart.number, data);
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
static bool receiveString(char* buf, unsigned int size, TickType_t timeout) {
	TickType_t ticksOld = xTaskGetTickCount();
	bool timedOut = false;
	bool receivedCrlf = false;

	unsigned int i = 0;

	while (i < size && !receivedCrlf) {
		while ( usart_get_flag(usart.number, USART_SR_RXNE) == 0 ) {
			taskYIELD();

			timedOut = timeout > 0 && xTaskGetTickCount() - ticksOld > timeout;
			if (timedOut) {
				break;
			}
		}

		buf[i] = usart_recv(usart.number);

		if (i > 0) {
			receivedCrlf = buf[i-1] == '\r' && buf[i] == '\n';
			if (receivedCrlf) {
				buf[i-1] = '\0';
			}
		}

		i++;
	}

	return receivedCrlf;
}

/**
 * Receives a certain number of bytes in the specified timeout period.
 *
 * @param buf[out] The buffer to store the data into.
 * @param size The number of bytes to receive.
 * @param usart The USART context to receive data from.
 * @param timeout The amount of time to wait before returning. If set to 0 then the function
 * waits indefinetly.
 *
 * @return The number of bytes received within the timeout period.
 */
static unsigned int receiveData(char* buf, unsigned int size, TickType_t timeout) {
	TickType_t ticksOld = xTaskGetTickCount();
	bool timedOut = false;

	unsigned int bytesRead = 0;

	while (bytesRead < size) {
		while ( usart_get_flag(usart.number, USART_SR_RXNE) == 0 ) {
			taskYIELD();

			timedOut = timeout > 0 && xTaskGetTickCount() - ticksOld > timeout;
			if (timedOut) {
				break;
			}
		}

		buf[bytesRead] = usart_recv(usart.number);
		bytesRead++;
	}

	return bytesRead;
}

static bool initProtocol(InfoPacket* info) {
	bool validResponse = false;
	char recv[16];
	char bufSize[] = {IN_BUF_SIZE >> 8, IN_BUF_SIZE & 0xFF};

	while(!validResponse) {
		receiveString(recv, 16, 0);
		validResponse = strcmp(recv, "ready") == 0;
	}
	
	sendString("bfsz\r\n");
	sendData(bufSize, 2);

	if (receiveString(recv, 16, 2000)) {
		validResponse = strcmp(recv, "info") == 0;

		if (!validResponse) {
			sendString("bad\r\n");
			return validResponse;
		}

		// Host is expected to send proper number of bytes
		// in the correct order.
		unsigned int expectedBytes = 8;
		validResponse = receiveData(recv, expectedBytes, 2000) == expectedBytes;

		if (!validResponse) {
			sendString("bad\r\n");
			return validResponse;
		}

		memcpy(info, recv, 8);
		sendString("good\r\n");
	}

	return validResponse;
}

void hostIOTask(void* args) {
	InfoPacket info;

	for (;;) {
		initProtocol(&info);
		
		while(1);	
	}
}

