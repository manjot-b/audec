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
		xQueueReceive(taskData.decoderQueue, &info, 0);
		while(1) {
			taskYIELD();
		}
	}
}
