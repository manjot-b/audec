#include "dac.h"

#include <stdint.h>

#include <FreeRTOS.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

#include "buffers.h"
#include "taskdata.h"

/** The input frequency of the timer from the APB. */
#define TIMER_FREQ (rcc_apb1_frequency * 2)

/** The target frequency for the timer. */
#define TIMER_TARGET_FREQ 1000000

/** Write to channel A. */
#define MCP_DAC_CRBIT_WRITE_A 		0

/** Unbufferd input. */
#define MCP_DAC_CRBIT_UNBUFFERED	0

/** Gain x2 on the output. */
#define MCP_DAC_CRBIT_GAIN_X2 		0

/** Shutdown the selected channel. */
#define MCP_DAC_CRBIT_SHUTDOWN 		0

/** Write to channel B. */
#define MCP_DAC_CRBIT_WRITE_B 		1 << 15

/** Bufferd input. */
#define MCP_DAC_CRBIT_BUFFERED 		1 << 14

/** Gain x1 on the output. */
#define MCP_DAC_CRBIT_GAIN_X1 		1 << 13

/** Activate the selected channel. */
#define MCP_DAC_CRBIT_ACTIVE 		1 << 12

static void sendToDAC(const InfoPacket* info);
static void mpcDACWrite(uint32_t spi, uint16_t data);
static uint16_t disableSPI(uint32_t spi);

void dacSetup(void) {
	// Timer setup
	rcc_periph_clock_enable(RCC_TIM2);
	timer_disable_counter(TIM2);
	rcc_periph_reset_pulse(RST_TIM2);

	timer_set_mode(TIM2,
		TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE,
		TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM2, (TIMER_FREQ / TIMER_TARGET_FREQ) - 1);
	timer_enable_preload(TIM2);
	timer_one_shot_mode(TIM2);
	timer_update_on_overflow(TIM2);
	timer_enable_update_event(TIM2);

	timer_enable_irq(TIM2, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM2_IRQ);

	// SPI setup
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_SPI1);

	gpio_set_mode(
		GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO4 | GPIO5 | GPIO7			// NSS=PA4, SCK=PA5, MOSI=PA7
	);
	spi_reset(SPI1); 
	spi_init_master(
		SPI1,
		SPI_CR1_BAUDRATE_FPCLK_DIV_4,
		SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_1,
		SPI_CR1_DFF_16BIT,
		SPI_CR1_MSBFIRST
	);
	spi_disable_software_slave_management(SPI1);
	spi_enable_ss_output(SPI1);

	// Clear the DAC voltage.
	spi_enable(SPI1);
	uint16_t command =
		MCP_DAC_CRBIT_WRITE_A |
		MCP_DAC_CRBIT_UNBUFFERED |
		MCP_DAC_CRBIT_GAIN_X1 |
		MCP_DAC_CRBIT_ACTIVE;
	//command |= (1 << (12)) - 1;
	spi_send(SPI1, command);
	spi_clean_disable(SPI1);
}

/** ISR for timer 2. Notifies the dacTask after an overflow. */
void tim2_isr(void) {
	if (timer_get_flag(TIM2, TIM_SR_UIF)) {
		timer_clear_flag(TIM2, TIM_SR_UIF);
		gpio_toggle(GPIOC, GPIO13);

		BaseType_t woken = pdFALSE;	
		vTaskNotifyGiveFromISR(taskData.dacHandle, &woken);
		portYIELD_FROM_ISR(woken);
	}
}

/**
 * Performs a blocking write to the DAC.
 *
 * @note This functions enables and disables the spi before and after the
 * write. For some reason disabling is required after each transfer.
 */
static void mpcDACWrite(uint32_t spi, uint16_t data) {
	spi_enable(spi);
	while ( !(SPI_SR(spi) & SPI_SR_TXE) ) {
		taskYIELD();
	}
	spi_write(spi, data);
	disableSPI(spi);
}

/**
 * Disables the spi and returns once all data has been sent and received.
 *
 * @return The last data read.
 */
static uint16_t disableSPI(uint32_t spi) {
	while ( !(SPI_SR(spi) & SPI_SR_RXNE) ) {
		taskYIELD();
	}
	uint16_t data = SPI_DR(spi);

	while ( !(SPI_SR(spi) & SPI_SR_TXE) ) {
		taskYIELD();
	}

	while ( (SPI_SR(spi) & SPI_SR_BSY) ) {
		taskYIELD();
	}

	spi_disable(spi);
	return data;
}

/**
 * Sends the audio data to the external DAC via SPI.
 *
 * @param info The info packet describing how to interpret the data in @c g_outputBuf.
 */
static void sendToDAC(const InfoPacket* info) {
	uint32_t period = TIMER_TARGET_FREQ / info->sampleRate;
	timer_set_period(TIM2, period - 1);

	int count = info->dataLength / (info->bitDepth / 8);

	for(int i = 0; i < count; i += info->channels) {
		// Write to first channel
		uint16_t command =
			MCP_DAC_CRBIT_WRITE_A |
			MCP_DAC_CRBIT_UNBUFFERED |
			MCP_DAC_CRBIT_GAIN_X1 |
			MCP_DAC_CRBIT_ACTIVE;
		command |= g_outputBuf[i];
		mpcDACWrite(SPI1, command);

		// Write to second channel
		command =
			MCP_DAC_CRBIT_WRITE_B |
			MCP_DAC_CRBIT_UNBUFFERED |
			MCP_DAC_CRBIT_GAIN_X1 |
			MCP_DAC_CRBIT_ACTIVE;
		if (info->channels > 1) { command |= g_outputBuf[i + 1]; }
		else { command |= g_outputBuf[i]; }
		mpcDACWrite(SPI1, command);

		timer_enable_counter(TIM2);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
}

void dacTask(void*) {
	InfoPacket info;

	for(;;) {
		xTaskNotifyGiveIndexed(taskData.decoderHandle, DECODER_NOTIFICATION_DAC);

		xQueueReceive(taskData.dacQueue, &info, portMAX_DELAY);
		sendToDAC(&info);
	}
}
