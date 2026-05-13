#include "telemetry/telemetry_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_state.h"

static void update_timestamp_locked(telemetry_data_t *telemetry)
{
    telemetry->timestamp_ms = app_state_now_ms();
}

void telemetry_manager_rebuild_locked(app_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->telemetry.attitude = state->manual_telemetry.attitude;
    state->telemetry.barometric_altitude = state->manual_telemetry.barometric_altitude;
    state->telemetry.battery = state->manual_telemetry.battery;
    state->telemetry.flight_mode = state->manual_telemetry.flight_mode;

    switch (state->gps_source) {
        case GPS_SOURCE_MANUAL:
            state->telemetry.gps = state->manual_telemetry.gps;
            state->telemetry.gps.source = GPS_SOURCE_MANUAL;
            break;
        case GPS_SOURCE_EXTERNAL_GPS:
            state->telemetry.gps = state->gps_status.last_fix;
            state->telemetry.gps.source = GPS_SOURCE_EXTERNAL_GPS;
            break;
        case GPS_SOURCE_SIMULATOR:
            state->telemetry.gps = state->simulator.current_gps;
            state->telemetry.gps.source = GPS_SOURCE_SIMULATOR;
            break;
        default:
            break;
    }

    state->ui.gps_source = state->gps_source;
    update_timestamp_locked(&state->telemetry);
}

void telemetry_manager_set_gps_source_locked(app_state_t *state, gps_source_t source)
{
    if (state == NULL) {
        return;
    }

    state->gps_source = source;
    state->ui.gps_source = source;
    telemetry_manager_rebuild_locked(state);
}

void telemetry_manager_apply_external_gps_locked(app_state_t *state, const gps_telemetry_t *gps)
{
    if ((state == NULL) || (gps == NULL)) {
        return;
    }

    state->gps_status.last_fix = *gps;
    state->gps_status.last_fix.source = GPS_SOURCE_EXTERNAL_GPS;
    state->gps_status.last_update_age_ms = 0;

    if (state->gps_source == GPS_SOURCE_EXTERNAL_GPS) {
        state->telemetry.gps = state->gps_status.last_fix;
        update_timestamp_locked(&state->telemetry);
    }
}

void telemetry_manager_apply_simulator_gps_locked(app_state_t *state, const gps_telemetry_t *gps)
{
    if ((state == NULL) || (gps == NULL)) {
        return;
    }

    state->simulator.current_gps = *gps;
    state->simulator.current_gps.source = GPS_SOURCE_SIMULATOR;

    if (state->gps_source == GPS_SOURCE_SIMULATOR) {
        state->telemetry.gps = state->simulator.current_gps;
        update_timestamp_locked(&state->telemetry);
    }
}

void telemetry_manager_handle_simulator_command_locked(app_state_t *state, simulator_command_t command)
{
    if (state == NULL) {
        return;
    }

    switch (command) {
        case SIMULATOR_COMMAND_START:
            state->simulator.state = SIM_RUNNING;
            state->simulator.current_gps.latitude = state->simulator.parameters.start_latitude;
            state->simulator.current_gps.longitude = state->simulator.parameters.start_longitude;
            state->simulator.current_gps.gps_altitude_m = state->simulator.parameters.altitude_m;
            state->simulator.current_gps.ground_speed_kmh = state->simulator.parameters.speed_kmh;
            state->simulator.current_gps.heading_deg = state->simulator.parameters.heading_deg;
            state->simulator.current_gps.satellites = state->simulator.parameters.satellites;
            state->simulator.current_gps.fix_valid = state->simulator.parameters.fix_valid;
            state->simulator.current_gps.source = GPS_SOURCE_SIMULATOR;
            break;
        case SIMULATOR_COMMAND_PAUSE:
            if (state->simulator.state == SIM_RUNNING) {
                state->simulator.state = SIM_PAUSED;
            }
            break;
        case SIMULATOR_COMMAND_RESUME:
            if (state->simulator.state == SIM_PAUSED) {
                state->simulator.state = SIM_RUNNING;
            }
            break;
        case SIMULATOR_COMMAND_STOP:
            state->simulator.state = SIM_STOPPED;
            break;
        case SIMULATOR_COMMAND_RESET:
            state->simulator.current_gps.latitude = state->simulator.parameters.start_latitude;
            state->simulator.current_gps.longitude = state->simulator.parameters.start_longitude;
            state->simulator.current_gps.gps_altitude_m = state->simulator.parameters.altitude_m;
            state->simulator.current_gps.heading_deg = state->simulator.parameters.heading_deg;
            state->simulator.current_gps.ground_speed_kmh = state->simulator.parameters.speed_kmh;
            state->simulator.current_gps.satellites = state->simulator.parameters.satellites;
            state->simulator.current_gps.fix_valid = state->simulator.parameters.fix_valid;
            state->simulator.current_gps.source = GPS_SOURCE_SIMULATOR;
            break;
        default:
            break;
    }

    state->simulator.next_emit_in_ms = (int32_t) state->simulator.parameters.update_interval_ms;
    telemetry_manager_rebuild_locked(state);
}
