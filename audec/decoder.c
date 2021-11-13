/**
 * Module to decode data from input buffer and place decoded data in
 * the output buffer.
 */

#include <FreeRTOS.h>
#include <queue.h>

#include "decoder.h"
#include "buffers.h"
#include "taskdata.h"

void decoderTask(void*) {
	InfoPacket info;

	for(;;) {
		xTaskNotifyGiveIndexed(taskData.hostIOHandle, HOSTIO_NOTIFICATION_DECODER);
		xQueueReceive(taskData.decoderQueue, &info, portMAX_DELAY);
		while(1) {
			taskYIELD();
		}
	}
}
