/**
 * Initialze and start all tasks.
 */
#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "hostio.h"

TaskHandle_t hostIOHandle;

void vApplicationStackOverflowHook(
  TaskHandle_t pxTask __attribute((unused)),
  portCHAR *pcTaskName __attribute((unused))
) {
	for(;;);	// Loop forever here..
}

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	hostIOSetup();

	xTaskCreate(hostIOTask, "hostio", 500, NULL, configMAX_PRIORITIES-1, &hostIOHandle);
	vTaskStartScheduler();

	for (;;);
	return 0;
}

