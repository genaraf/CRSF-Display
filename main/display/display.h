#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * LCD display driver for M5Stack Basic (ILI9341 320x240 over SPI3).
 *
 * display_init() must be called early in the bootstrap, before any other
 * subsystem that uses SPI3 (in particular sdcard_json), because it
 * configures the shared SPI bus with a transfer size big enough for the
 * LVGL flush buffer.
 *
 * After display_init() returns ESP_OK, LVGL is fully running on its own
 * task. Code that touches LVGL widgets MUST take the display mutex via
 * display_lock() / display_unlock().
 */

esp_err_t display_init(void);

/* Take/release the LVGL access mutex. Returns true on success. */
bool display_lock(uint32_t timeout_ms);
void display_unlock(void);

/* Logical display dimensions used by the UI layer. */
#define DISPLAY_WIDTH  320
#define DISPLAY_HEIGHT 240

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */
