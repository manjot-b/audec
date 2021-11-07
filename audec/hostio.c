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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

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

/**
 * DMA Context
 */
typedef struct {
	enum rcc_periph_clken rccDma;
	uint8_t irq;
	uint32_t number;
	uint8_t channel;
	uint32_t peripheralAddr;
} DmaContext;

static DmaContext dma = {
	.rccDma				= RCC_DMA1,
	.irq				= NVIC_DMA1_CHANNEL3_IRQ,
	.number				= DMA1,
	.channel			= DMA_CHANNEL3,
	.peripheralAddr		= (uint32_t)&USART3_DR,
};

static bool initProtocol(volatile InfoPacket* info);
static void copyToInfoStruct(volatile InfoPacket* info, char* data);
static void restoreDma(uint16_t size);
static void sendString(const char* str);
static void sendData(const char* data, uint32_t size);
static void sendChar(char data);
static bool receiveString(char* buf, unsigned int size, TickType_t timeout);
static unsigned int receiveData(char* buf, unsigned int size, TickType_t timeout);

#define IN_BUF_SIZE 256
static volatile uint8_t inputBuf[IN_BUF_SIZE];

void hostIOSetup(void) {
	// Setup USART with hardware flow control.
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

	// Enable DMA irq
	rcc_periph_clock_enable(dma.rccDma);
	nvic_set_priority(dma.irq, 0);
	nvic_enable_irq(dma.irq);

	// Confifure DMA
	dma_disable_channel(dma.number, dma.channel);
	dma_channel_reset(dma.number, dma.channel);
	dma_set_peripheral_address(dma.number, dma.channel, dma.peripheralAddr);
	dma_set_memory_address(dma.number, dma.channel, (uint32_t)inputBuf);

	dma_set_peripheral_size(dma.number, dma.channel, DMA_CCR_MSIZE_8BIT);
	dma_set_memory_size(dma.number, dma.channel, DMA_CCR_MSIZE_8BIT);

	dma_enable_memory_increment_mode(dma.number, dma.channel);
	dma_set_read_from_peripheral(dma.number, dma.channel);
	dma_set_priority(dma.number, dma.channel, DMA_CCR_PL_HIGH);
	dma_enable_transfer_complete_interrupt(dma.number, dma.channel);
}

static void restoreDma(uint16_t size) {
	dma_disable_channel(dma.number, dma.channel);
	usart_disable_rx_dma(usart.number);

	dma_set_number_of_data(dma.number, dma.channel, size);

	usart_enable_rx_dma(usart.number);
	dma_enable_channel(dma.number, dma.channel);
}

void dma1_channel3_isr(void) {
	if (dma_get_interrupt_flag(dma.number, dma.channel, DMA_TCIF)) {
		dma_clear_interrupt_flags(dma.number, dma.channel, DMA_TCIF);
	}

	BaseType_t woken = pdFALSE;	
	vTaskNotifyGiveFromISR(hostIOHandle, &woken);
	portYIELD_FROM_ISR(woken);
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
				return receivedCrlf;
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
				return bytesRead;
			}
		}

		buf[bytesRead] = usart_recv(usart.number);
		bytesRead++;
	}

	return bytesRead;
}

static bool initProtocol(volatile InfoPacket* info) {
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
			return validResponse;
		}

		// Host is expected to send proper number of bytes
		// in the correct order.
		unsigned int expectedBytes = 8;
		validResponse = receiveData(recv, expectedBytes, 2000) == expectedBytes;

		if (!validResponse) {
			return validResponse;
		}

		copyToInfoStruct(info, recv);
	}

	return validResponse;
}

static void copyToInfoStruct(volatile InfoPacket* info, char* data) {
	info->sampleRate |= data[0] << 24;
	info->sampleRate |= data[1] << 16;
	info->sampleRate |= data[2] << 8;
	info->sampleRate |= data[3];
	info->bitDepth = *((uint8_t*)(&data[4]));
	info->channels = *((uint8_t*)(&data[5]));
	info->dataLength |= data[6] << 8;
	info->dataLength |= data[7];
}

void hostIOTask(void*) {
	volatile InfoPacket info;

	for (;;) {
		if (initProtocol(&info)) {
			restoreDma(info.dataLength);
			sendString("ready\r\n");
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
		
		while(1);	
	}
}

