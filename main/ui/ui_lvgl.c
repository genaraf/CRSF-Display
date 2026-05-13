#include "ui/ui_lvgl.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_state.h"
#include "display/display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

static const char *TAG = "ui_lvgl";

#define UI_COLOR_BG       lv_color_hex(0x08110D)
#define UI_COLOR_PANEL    lv_color_hex(0x132D24)
#define UI_COLOR_PANEL_2  lv_color_hex(0x10241B)
#define UI_COLOR_LINE     lv_color_hex(0x245246)
#define UI_COLOR_ACCENT   lv_color_hex(0xFFD36E)
#define UI_COLOR_ACCENT2  lv_color_hex(0x82E6D2)
#define UI_COLOR_TEXT     lv_color_hex(0xF9F1DB)
#define UI_COLOR_MUTED    lv_color_hex(0x7CA599)
#define UI_COLOR_BAD      lv_color_hex(0xFF737C)
#define UI_COLOR_OK       lv_color_hex(0x90F0A5)

#define UI_HEADER_H 24
#define UI_FOOTER_H 22
#define UI_GAP 6
#define UI_CARD_RADIUS 8

static lv_obj_t *s_root;
static lv_obj_t *s_title;
static lv_obj_t *s_status;
static lv_obj_t *s_content;
static lv_obj_t *s_footer;
static ui_screen_id_t s_last_screen = -1;
static app_state_t s_snapshot;

static lv_obj_t *s_channel_name[APP_RX_CHANNEL_COUNT];
static lv_obj_t *s_channel_value[APP_RX_CHANNEL_COUNT];
static lv_obj_t *s_channel_bar[APP_RX_CHANNEL_COUNT];
static lv_obj_t *s_card_label[12];
static lv_obj_t *s_card_value[12];

static const char *screen_title(ui_screen_id_t screen_id)
{
    switch (screen_id) {
        case SCREEN_CRSF_CHANNELS: return "CRSF Channels";
        case SCREEN_LINK_STATISTICS: return "Link Statistics";
        case SCREEN_TELEMETRY_OVERVIEW: return "Telemetry";
        case SCREEN_MANUAL_TELEMETRY: return "Manual GPS";
        case SCREEN_SIMULATION_PARAMETERS: return "Simulation";
        case SCREEN_GPS_STATUS: return "GPS Status";
        default: return "CRSF Display";
    }
}

static const char *short_gps_source(gps_source_t source)
{
    switch (source) {
        case GPS_SOURCE_MANUAL: return "Manual";
        case GPS_SOURCE_EXTERNAL_GPS: return "GPS";
        case GPS_SOURCE_SIMULATOR: return "Sim";
        default: return "?";
    }
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, color, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    return label;
}

static lv_obj_t *make_panel(lv_obj_t *parent, int32_t x, int32_t y, int32_t w, int32_t h)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_style_bg_color(panel, UI_COLOR_PANEL, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, UI_COLOR_LINE, LV_PART_MAIN);
    lv_obj_set_style_border_opa(panel, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(panel, UI_CARD_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 5, LV_PART_MAIN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    return panel;
}

static void clear_content(void)
{
    lv_obj_clean(s_content);
    memset(s_channel_name, 0, sizeof(s_channel_name));
    memset(s_channel_value, 0, sizeof(s_channel_value));
    memset(s_channel_bar, 0, sizeof(s_channel_bar));
    memset(s_card_label, 0, sizeof(s_card_label));
    memset(s_card_value, 0, sizeof(s_card_value));
}

static void make_channel_layout(void)
{
    clear_content();
    const int w = 72;
    const int h = 38;
    for (int i = 0; i < APP_RX_CHANNEL_COUNT; ++i) {
        int col = i % 4;
        int row = i / 4;
        lv_obj_t *card = make_panel(s_content, col * (w + UI_GAP), row * (h + UI_GAP), w, h);

        s_channel_name[i] = make_label(card, &lv_font_montserrat_12, UI_COLOR_ACCENT2);
        lv_obj_set_pos(s_channel_name[i], 0, -1);
        lv_obj_set_width(s_channel_name[i], 30);

        s_channel_value[i] = make_label(card, &lv_font_montserrat_14, UI_COLOR_TEXT);
        lv_obj_set_pos(s_channel_value[i], 32, -3);
        lv_obj_set_width(s_channel_value[i], 36);

        s_channel_bar[i] = lv_bar_create(card);
        lv_obj_set_pos(s_channel_bar[i], 0, 24);
        lv_obj_set_size(s_channel_bar[i], w - 10, 5);
        lv_bar_set_range(s_channel_bar[i], -100, 100);
        lv_obj_set_style_bg_color(s_channel_bar[i], lv_color_hex(0x24342D), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_channel_bar[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_channel_bar[i], UI_COLOR_ACCENT, LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_channel_bar[i], 99, LV_PART_MAIN);
        lv_obj_set_style_radius(s_channel_bar[i], 99, LV_PART_INDICATOR);
    }
}

static void make_cards_layout(int count)
{
    clear_content();
    for (int i = 0; i < count && i < 12; ++i) {
        int col = i % 2;
        int row = i / 2;
        lv_obj_t *card = make_panel(s_content, col * 153, row * 34, 147, 30);
        s_card_label[i] = make_label(card, &lv_font_montserrat_12, UI_COLOR_ACCENT2);
        lv_obj_set_pos(s_card_label[i], 0, -3);
        lv_obj_set_width(s_card_label[i], 136);
        s_card_value[i] = make_label(card, &lv_font_montserrat_12, UI_COLOR_TEXT);
        lv_obj_set_pos(s_card_value[i], 0, 12);
        lv_obj_set_width(s_card_value[i], 136);
    }
}

static void set_card(int idx, const char *label, const char *fmt, ...)
{
    if (idx < 0 || idx >= 12 || s_card_label[idx] == NULL || s_card_value[idx] == NULL) {
        return;
    }
    char value[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(value, sizeof(value), fmt, args);
    va_end(args);
    lv_label_set_text(s_card_label[idx], label);
    lv_label_set_text(s_card_value[idx], value);
}

static void update_header_footer(const app_state_t *state)
{
    char status[48];
    char footer[80];
    snprintf(status, sizeof(status), "%s | %s", state->crsf_link.link_up ? "LINK UP" : "LINK DOWN", short_gps_source(state->gps_source));
    snprintf(footer, sizeof(footer), "A Back   B Select   C Next   %s", state->ui.mode == UI_MODE_EDIT ? "EDIT" : "VIEW");

    lv_label_set_text(s_title, screen_title(state->ui.screen_id));
    lv_label_set_text(s_status, status);
    lv_label_set_text(s_footer, footer);
    lv_obj_set_style_text_color(s_status, state->crsf_link.link_up ? UI_COLOR_OK : UI_COLOR_BAD, LV_PART_MAIN);
}

static void render_channels(const app_state_t *state)
{
    if (s_last_screen != SCREEN_CRSF_CHANNELS || s_channel_name[0] == NULL) {
        make_channel_layout();
    }
    for (int i = 0; i < APP_RX_CHANNEL_COUNT; ++i) {
        char value[16];
        int pct = (int)(state->rx_channels[i].normalized_value * 100.0f);
        if (pct > 100) pct = 100;
        if (pct < -100) pct = -100;
        snprintf(value, sizeof(value), "%+d", pct);
        lv_label_set_text(s_channel_name[i], state->rx_channels[i].name);
        lv_label_set_text(s_channel_value[i], value);
        lv_bar_set_value(s_channel_bar[i], pct, LV_ANIM_OFF);
    }
}

static void render_link_stats(const app_state_t *state)
{
    if (s_last_screen != SCREEN_LINK_STATISTICS || s_card_label[0] == NULL) {
        make_cards_layout(10);
    }
    set_card(0, "RSSI ANT1", "-%d dBm", state->link_statistics.uplink_rssi_ant1_dbm_neg);
    set_card(1, "RSSI ANT2", "-%d dBm", state->link_statistics.uplink_rssi_ant2_dbm_neg);
    set_card(2, "UPLINK LQ", "%d %%", state->link_statistics.uplink_lq_percent);
    set_card(3, "UPLINK SNR", "%d dB", state->link_statistics.uplink_snr_db);
    set_card(4, "ANTENNA", "%u", state->link_statistics.active_antenna);
    set_card(5, "RF MODE", "%u", state->link_statistics.rf_mode);
    set_card(6, "TX POWER", "%u", state->link_statistics.uplink_tx_power);
    set_card(7, "DOWN RSSI", "-%d dBm", state->link_statistics.downlink_rssi_dbm_neg);
    set_card(8, "DOWN LQ", "%d %%", state->link_statistics.downlink_lq_percent);
    set_card(9, "DOWN SNR", "%d dB", state->link_statistics.downlink_snr_db);
}

static void render_telemetry(const app_state_t *state)
{
    const gps_telemetry_t *gps = &state->telemetry.gps;
    if (s_last_screen != SCREEN_TELEMETRY_OVERVIEW || s_card_label[0] == NULL) {
        make_cards_layout(10);
    }
    set_card(0, "GPS SOURCE", "%s", short_gps_source(state->gps_source));
    set_card(1, "GPS FIX", "%s / %u sats", gps->fix_valid ? "YES" : "NO", gps->satellites);
    set_card(2, "LATITUDE", "%.6f", gps->latitude);
    set_card(3, "LONGITUDE", "%.6f", gps->longitude);
    set_card(4, "ALTITUDE", "%.1f m", (double)gps->gps_altitude_m);
    set_card(5, "SPEED", "%.1f km/h", (double)gps->ground_speed_kmh);
    set_card(6, "HEADING", "%.0f deg", (double)gps->heading_deg);
    set_card(7, "BATTERY", "%.1f V %.1f A", (double)state->telemetry.battery.voltage_v, (double)state->telemetry.battery.current_a);
    set_card(8, "BARO", "%.1f m", (double)state->telemetry.barometric_altitude.baro_altitude_m);
    set_card(9, "FLIGHT MODE", "%s", state->telemetry.flight_mode.mode_name);
}

static void render_manual(const app_state_t *state)
{
    const gps_telemetry_t *gps = &state->manual_telemetry.gps;
    if (s_last_screen != SCREEN_MANUAL_TELEMETRY || s_card_label[0] == NULL) {
        make_cards_layout(8);
    }
    set_card(0, "FIX", "%s", gps->fix_valid ? "YES" : "NO");
    set_card(1, "SATELLITES", "%u", gps->satellites);
    set_card(2, "LATITUDE", "%.6f", gps->latitude);
    set_card(3, "LONGITUDE", "%.6f", gps->longitude);
    set_card(4, "ALTITUDE", "%.1f m", (double)gps->gps_altitude_m);
    set_card(5, "SPEED", "%.1f km/h", (double)gps->ground_speed_kmh);
    set_card(6, "HEADING", "%.0f deg", (double)gps->heading_deg);
    set_card(7, "SELECTED", "%s", state->ui.selected_item);
}

static void render_simulation(const app_state_t *state)
{
    if (s_last_screen != SCREEN_SIMULATION_PARAMETERS || s_card_label[0] == NULL) {
        make_cards_layout(10);
    }
    set_card(0, "STATE", "%s", app_state_simulator_state_to_string(state->simulator.state));
    set_card(1, "PROFILE", "%s", state->simulator.active_profile_name);
    set_card(2, "START LAT", "%.6f", state->simulator.parameters.start_latitude);
    set_card(3, "START LON", "%.6f", state->simulator.parameters.start_longitude);
    set_card(4, "ALTITUDE", "%.1f m", (double)state->simulator.parameters.altitude_m);
    set_card(5, "SPEED", "%.1f km/h", (double)state->simulator.parameters.speed_kmh);
    set_card(6, "HEADING", "%.0f deg", (double)state->simulator.parameters.heading_deg);
    set_card(7, "INTERVAL", "%lu ms", (unsigned long)state->simulator.parameters.update_interval_ms);
    set_card(8, "SDCARD", "%s", state->sdcard.mounted ? "mounted" : "not mounted");
    set_card(9, "JSON", "%s", app_state_json_operation_status_to_string(state->simulator.last_json_operation.status));
}

static void render_gps_status(const app_state_t *state)
{
    const gps_status_t *gps = &state->gps_status;
    if (s_last_screen != SCREEN_GPS_STATUS || s_card_label[0] == NULL) {
        make_cards_layout(10);
    }
    set_card(0, "CONNECTED", "%s", gps->connected ? "YES" : "NO");
    set_card(1, "STREAM", "%s", gps->data_stream_present ? "YES" : "NO");
    set_card(2, "PROTOCOL", "%s", gps->protocol_name);
    set_card(3, "SERIAL", "%s", gps->serial_type_name);
    set_card(4, "RX/TX", "%d/%d", gps->rx_gpio, gps->tx_gpio);
    set_card(5, "BAUD", "%d", gps->baud_rate);
    set_card(6, "FIX", "%s", gps->last_fix.fix_valid ? "YES" : "NO");
    set_card(7, "SATELLITES", "%u", gps->last_fix.satellites);
    set_card(8, "AGE", "%lld ms", (long long)gps->last_update_age_ms);
    set_card(9, "ERRORS", "%lu", (unsigned long)gps->parser_errors);
}

static void render_snapshot(const app_state_t *state)
{
    update_header_footer(state);

    switch (state->ui.screen_id) {
        case SCREEN_CRSF_CHANNELS: render_channels(state); break;
        case SCREEN_LINK_STATISTICS: render_link_stats(state); break;
        case SCREEN_TELEMETRY_OVERVIEW: render_telemetry(state); break;
        case SCREEN_MANUAL_TELEMETRY: render_manual(state); break;
        case SCREEN_SIMULATION_PARAMETERS: render_simulation(state); break;
        case SCREEN_GPS_STATUS: render_gps_status(state); break;
        default: break;
    }

    s_last_screen = state->ui.screen_id;
}

static void ui_lvgl_task(void *arg)
{
    (void)arg;
    while (true) {
        app_state_get_snapshot(&s_snapshot);
        if (display_lock(1000)) {
            render_snapshot(&s_snapshot);
            display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(APP_UI_REFRESH_MS));
    }
}

esp_err_t ui_lvgl_init(void)
{
    if (!display_lock(1000)) {
        ESP_LOGE(TAG, "display_lock timeout");
        return ESP_ERR_TIMEOUT;
    }

    lv_obj_t *screen = lv_screen_active();
    lv_obj_clean(screen);
    lv_obj_set_style_bg_color(screen, UI_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);

    s_root = lv_obj_create(screen);
    lv_obj_set_size(s_root, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_obj_set_style_bg_color(s_root, UI_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_root, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_root, 8, LV_PART_MAIN);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    s_title = make_label(s_root, &lv_font_montserrat_14, UI_COLOR_ACCENT);
    lv_obj_set_pos(s_title, 0, 0);
    lv_obj_set_width(s_title, 190);

    s_status = make_label(s_root, &lv_font_montserrat_12, UI_COLOR_MUTED);
    lv_obj_set_pos(s_status, 195, 2);
    lv_obj_set_width(s_status, 110);
    lv_obj_set_style_text_align(s_status, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    s_content = lv_obj_create(s_root);
    lv_obj_set_pos(s_content, 0, UI_HEADER_H);
    lv_obj_set_size(s_content, DISPLAY_WIDTH - 16, DISPLAY_HEIGHT - UI_HEADER_H - UI_FOOTER_H - 16);
    lv_obj_set_style_bg_opa(s_content, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_content, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_content, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_content, LV_OBJ_FLAG_SCROLLABLE);

    s_footer = make_label(s_root, &lv_font_montserrat_12, UI_COLOR_MUTED);
    lv_obj_set_pos(s_footer, 0, DISPLAY_HEIGHT - UI_FOOTER_H - 8);
    lv_obj_set_size(s_footer, DISPLAY_WIDTH - 16, UI_FOOTER_H);
    lv_obj_set_style_bg_color(s_footer, UI_COLOR_PANEL_2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_footer, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_text_align(s_footer, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    display_unlock();

    BaseType_t result = xTaskCreate(ui_lvgl_task, "task_ui_lvgl", APP_TASK_STACK_LARGE, NULL, APP_TASK_PRIORITY_NORMAL, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LVGL UI task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LVGL UI started");
    return ESP_OK;
}
