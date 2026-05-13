#ifndef TELEMETRY_MANAGER_H
#define TELEMETRY_MANAGER_H

#include "app_state.h"
#include "app_events.h"

void telemetry_manager_rebuild_locked(app_state_t *state);
void telemetry_manager_set_gps_source_locked(app_state_t *state, gps_source_t source);
void telemetry_manager_apply_external_gps_locked(app_state_t *state, const gps_telemetry_t *gps);
void telemetry_manager_apply_simulator_gps_locked(app_state_t *state, const gps_telemetry_t *gps);
void telemetry_manager_handle_simulator_command_locked(app_state_t *state, simulator_command_t command);

#endif
