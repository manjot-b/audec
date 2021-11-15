/**
 * Initialize and start all tasks.
 */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

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

	hostIOSetup();

	taskData.decoderQueue = xQueueCreate(10, sizeof(InfoPacket));
	xTaskCreate(hostIOTask, "hostio", 500, NULL, configMAX_PRIORITIES-1, &(taskData.hostIOHandle));
	xTaskCreate(decoderTask, "decoder", 200, NULL, configMAX_PRIORITIES-1, &(taskData.decoderHandle));
	vTaskStartScheduler();

	for (;;);
	return 0;
}

