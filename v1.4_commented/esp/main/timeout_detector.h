#ifndef TIMEOUT_DETECTOR_H
#define TIMEOUT_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

/* timeout_detector_init
 * Initialises the software timeout detector with a specified
 * timeout period. Records the current hardware timer value as
 * the reference point.
 *
 * timeout_us : timeout period in microseconds.
 *              Example: 500000 µs = 0.5 seconds.
 *
 * The timeout is checked by timeout_detector_triggered().
 * The timer is reset by timeout_detector_reset().
 */
void timeout_detector_init(uint64_t timeout_us);

/* timeout_detector_reset
 * Resets the timeout counter to the current hardware time.
 * Call this every time a valid command packet is received to
 * extend the deadline.
 */
void timeout_detector_reset(void);

/* timeout_detector_triggered
 * Checks whether the timeout period has elapsed since the last
 * reset.
 *
 * Returns true if the elapsed time since the last reset exceeds
 * the configured timeout. Returns false if the timeout has not
 * yet expired.
 *
 * The caller (typically the main loop) should check this flag
 * and call failsafe_engage() if it returns true.
 */
bool timeout_detector_triggered(void);

#endif
