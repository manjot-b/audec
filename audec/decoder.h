#ifndef DECODER_H
#define DECODER_H

/**
 * The task to decode PCM data from the input buffer to the output buffer.
 *
 * Waits for a notification from @c hostIOTask that the input buffer @c g_inputBuf
 * is ready to be decoded. Then places the data in @g_outputBuf for the DAC.
 */
void decoderTask(void*);

#endif
