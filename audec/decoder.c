/**
 * Module to decode data from input buffer and place decoded data in
 * the output buffer.
 */

#include <stdint.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "decoder.h"
#include "buffers.h"
#include "taskdata.h"

static void decode(const InfoPacket* info);
static void decode16(const InfoPacket* info);

static void decode(const InfoPacket* info) {
	switch (info->bitDepth) {
	case 16:
		decode16(info);
		break;
	}
}

static void decode16(const InfoPacket* info) {
	for (size_t inp = 0, out = 0; inp < info->dataLength; inp += 2, out++) {
		int32_t spcm = g_inputBuf[inp] | (g_inputBuf[inp + 1] << 8);
		spcm += 1 << 15;
		uint16_t pcm = (uint16_t)spcm;

		// Only a 12-bit DAC
		pcm = pcm >> 4;
		g_outputBuf[out] = pcm;
	}
}

void decoderTask(void*) {
	InfoPacket info;

	for(;;) {
		xTaskNotifyGiveIndexed(taskData.hostIOHandle, HOSTIO_NOTIFICATION_DECODER);
		xQueueReceive(taskData.decoderQueue, &info, portMAX_DELAY);
		decode(&info);
	}
}
