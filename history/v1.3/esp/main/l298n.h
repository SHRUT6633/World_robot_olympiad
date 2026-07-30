#ifndef L298N_H
#define L298N_H

#include <stdint.h>
#include <stdbool.h>

void l298n_init(void);
void l298n_set_motor(int speed_pct, bool forward);

#endif
