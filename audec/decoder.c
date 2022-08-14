/**
 * Module to decode data from input buffer and place decoded data in
 * the output buffer.
 */

#include "decoder.h"

#include <stdint.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "buffers.h"
#include "taskdata.h"

static void decode(const InfoPacket* info);
static void decode8(const InfoPacket* info);
static void decode16(const InfoPacket* info);

static void decode(const InfoPacket* info) {
	switch (info->bitDepth) {
	case 8:
		decode8(info); break;
	case 16:
		decode16(info); break;
	}
}

static void decode8(const InfoPacket* info) {
	for (size_t i = 0; i < info->dataLength; ++i) {
		uint16_t pcm = g_inputBuf[i];

		// Scale because of 12-bit DAC
		pcm = pcm << 4;
		g_outputBuf[i] = pcm;
	}
}

static void decode16(const InfoPacket* info) {
	for (size_t inp = 0, out = 0; inp < info->dataLength; inp += 2, out++) {
		int16_t spcm = (g_inputBuf[inp + 1] << 8) | g_inputBuf[inp];
		int32_t overflow = spcm;
		overflow += 1 << 15;
		uint16_t pcm = (uint16_t)overflow;

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
		ulTaskNotifyTakeIndexed(DECODER_NOTIFICATION_DAC, pdTRUE, portMAX_DELAY);

		decode(&info);
		xQueueSend(taskData.dacQueue, &info, portMAX_DELAY);
	}
}
