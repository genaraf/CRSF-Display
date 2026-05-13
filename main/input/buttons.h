#ifndef INPUT_BUTTONS_H
#define INPUT_BUTTONS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 3-button driver for M5Stack Basic.
 *
 *  Btn A = GPIO39 (left)
 *  Btn B = GPIO38 (middle)
 *  Btn C = GPIO37 (right)
 *
 * All inputs are active-low with external pull-ups on the M5Stack PCB
 * (GPIO34..39 do not support internal pull-ups on ESP32).
 *
 * The driver runs a polling task with software debounce and posts
 * BTN_A_SHORT / BTN_B_SHORT / BTN_C_SHORT events to the global app_events
 * queue on release. Long-press detection can be added later without
 * breaking the API.
 */

esp_err_t buttons_start(void);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_BUTTONS_H */
