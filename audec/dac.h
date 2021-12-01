#ifndef DAC_H
#define DAC_H

/**
 * Setup the timers and spi for the external DAC
 */
void dacSetup(void);

/**
 * The task to communicate with the external DAC.
 *
 * Waits for a notification from the @c decoderTask that the @c outputBuf is ready
 * to be processed. Notifies the @c decoderTask when all data has sent to the DAC.
 */
void dacTask(void* args);

#endif
