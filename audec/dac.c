#include "dac.h"

#include <stdint.h>

#include <FreeRTOS.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

#include "taskdata.h"

/** The input frequency of the timer from the APB. */
#define TIMER_FREQ (rcc_apb1_frequency * 2)

/** The target frequency for the timer. */
#define TIMER_TARGET_FREQ 1000000

static void sendToDac(const InfoPacket* info);

void dacSetup(void) {
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
 * Sends the audio data to the external DAC via SPI.
 *
 * @param info The info packet describing how to interpret the data in @c g_outputBuf.
 */
static void sendToDac(const InfoPacket* info) {
	uint32_t period = TIMER_TARGET_FREQ / info->sampleRate;
	timer_set_period(TIM2, period - 1);

	int count = info->dataLength / (info->bitDepth / 8);
	for(int i = 0; i < count; i += info->channels) {
		timer_enable_counter(TIM2);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
}

void dacTask(void*) {
	InfoPacket info;

	for(;;) {
		xTaskNotifyGiveIndexed(taskData.decoderHandle, DECODER_NOTIFICATION_DAC);

		xQueueReceive(taskData.dacQueue, &info, portMAX_DELAY);
		sendToDac(&info);
	}
}
