#ifndef WATCHDOG_H
#define WATCHDOG_H

/* watchdog_init
 * Initialises the ESP32 Task Watchdog Timer (TWDT) with a 3-second
 * timeout. Adds the calling task to the set of watched tasks.
 * If the calling task fails to reset ("feed") the watchdog within
 * the timeout period, the system will panic and reboot
 * (trigger_panic = true).
 *
 * This provides a last-resort recovery mechanism if a task hangs.
 */
void watchdog_init(void);

/* watchdog_feed
 * Resets (feeds) the task watchdog timer for the current task.
 * Must be called periodically from the main task loop to prevent
 * a watchdog-induced system reset.
 *
 * The feeding interval should be significantly shorter than the
 * 3-second timeout (e.g., every 500–1000 ms).
 */
void watchdog_feed(void);

#endif
