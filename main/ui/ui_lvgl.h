#ifndef UI_LVGL_H
#define UI_LVGL_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * LVGL UI layer for the CRSF-Display.
 *
 * Phase 1 contract: ui_lvgl_init() builds a single bootstrap screen showing
 * "CRSF Display" centered on a dark background. It must be called AFTER
 * display_init() so that LVGL is already running. The function takes the
 * LVGL mutex internally.
 *
 * Later phases will extend this module with the six required screens
 * (CRSF Channels, Link Statistics, Telemetry Overview, Manual Telemetry,
 * Simulation Parameters, GPS Status) and an update function driven by
 * app_state snapshots.
 */

esp_err_t ui_lvgl_init(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_LVGL_H */
