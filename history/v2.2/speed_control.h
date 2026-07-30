#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

void speed_init(void);
void speed_set_target(int percent);
int speed_get_current(void);

#endif
