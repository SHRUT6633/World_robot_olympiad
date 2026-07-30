#ifndef TIMEOUT_DETECTOR_H
#define TIMEOUT_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

void timeout_detector_init(uint64_t timeout_us);
void timeout_detector_reset(void);
bool timeout_detector_triggered(void);

#endif
