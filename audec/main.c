/**
 * Initialize and start all tasks.
 */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "dac.h"
#include "decoder.h"
#include "hostio.h"
#include "taskdata.h"

TaskData taskData;

void vApplicationStackOverflowHook(
  TaskHandle_t pxTask __attribute((unused)),
  portCHAR *pcTaskName __attribute((unused))
) {
	for(;;);	// Loop forever here..
}

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	// Setup LED for debugging.
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);

	hostIOSetup();
	dacSetup();

	taskData.decoderQueue = xQueueCreate(4, sizeof(InfoPacket));
	taskData.dacQueue = xQueueCreate(4, sizeof(InfoPacket));

	xTaskCreate(hostIOTask, "hostio", 200, NULL, configMAX_PRIORITIES-2, &(taskData.hostIOHandle));
	xTaskCreate(decoderTask, "decoder", 200, NULL, configMAX_PRIORITIES-2, &(taskData.decoderHandle));
	xTaskCreate(dacTask, "dac", 200, NULL, configMAX_PRIORITIES-1, &(taskData.dacHandle));
	vTaskStartScheduler();

	for (;;);
	return 0;
}

