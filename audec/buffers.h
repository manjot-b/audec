#ifndef BUFFERS_H
#define BUFFERS_H

#include <stdint.h>

#define IN_BUF_SIZE 4096
extern volatile uint8_t g_inputBuf[IN_BUF_SIZE];

#define OUT_BUF_SIZE 4096
extern uint16_t g_outputBuf[OUT_BUF_SIZE];

#endif
