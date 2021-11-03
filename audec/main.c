/**
 * Initialze and start all tasks.
 */
#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "hostio.h"

static usart_ctx usart = {
	.rcc_gpio	= RCC_GPIOB,
	.rcc_usart	= RCC_USART3,
	.number		= USART3,
	.gpio		= GPIOB,
	.tx			= GPIO_USART3_TX,
	.rx			= GPIO_USART3_RX,
	.cts		= GPIO13,
	.rts		= GPIO14,
};

void vApplicationStackOverflowHook(
  TaskHandle_t pxTask __attribute((unused)),
  portCHAR *pcTaskName __attribute((unused))
) {
	for(;;);	// Loop forever here..
}

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	hostio_setup(&usart);

	xTaskCreate(hostio_task, "hostio", 500, (void*)&usart, configMAX_PRIORITIES-1, NULL);
	vTaskStartScheduler();

	for (;;);
	return 0;
}

