#ifndef TASKDATA_H
#define TASKDATA_h

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/**
 * Holds handles to tasks and their queues (if any).
 */
typedef struct {
	TaskHandle_t hostIOHandle;
	TaskHandle_t decoderHandle;
	QueueHandle_t decoderQueue;
} TaskData;

extern TaskData taskData;

/**
 * Holds the data about the chunk of audio data received.
 */
typedef struct {
	uint32_t sampleRate;
	uint8_t bitDepth;
	uint8_t channels;
	uint16_t dataLength;
} InfoPacket;

#endif
