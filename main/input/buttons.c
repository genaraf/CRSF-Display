#include "input/buttons.h"

#include <stddef.h>

#include "app_config.h"
#include "app_events.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "buttons";

#define BTN_PIN_A 39
#define BTN_PIN_B 38
#define BTN_PIN_C 37

#define BTN_POLL_PERIOD_MS 20
#define BTN_DEBOUNCE_MS    40

typedef struct {
    int pin;
    button_event_id_t short_event;
    int stable_level;     /* last debounced level (1 = released, 0 = pressed) */
    int last_raw_level;
    int64_t last_change_us;
    bool press_reported;
} button_state_t;

static button_state_t s_buttons[] = {
    {BTN_PIN_A, BTN_A_SHORT, 1, 1, 0, false},
    {BTN_PIN_B, BTN_B_SHORT, 1, 1, 0, false},
    {BTN_PIN_C, BTN_C_SHORT, 1, 1, 0, false},
};

static void post_button_event(button_event_id_t id)
{
    app_event_t event = {
        .type = APP_EVENT_BUTTON,
        .timestamp_ms = esp_timer_get_time() / 1000,
    };
    event.data.button_id = id;
    if (app_events_post(&event, 0) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to post button event %d", (int) id);
    }
}

static void poll_button(button_state_t *btn)
{
    int raw = gpio_get_level(btn->pin);
    int64_t now_us = esp_timer_get_time();

    if (raw != btn->last_raw_level) {
        btn->last_raw_level = raw;
        btn->last_change_us = now_us;
        return;
    }

    if (raw == btn->stable_level) {
        return;
    }

    if ((now_us - btn->last_change_us) < (BTN_DEBOUNCE_MS * 1000LL)) {
        return;
    }

    btn->stable_level = raw;
    if (raw == 0) {
        /* Press edge. Defer reporting until release to support long-press later. */
        btn->press_reported = false;
    } else {
        /* Release edge - emit short-press if it has not been emitted as long. */
        if (!btn->press_reported) {
            post_button_event(btn->short_event);
            btn->press_reported = true;
        }
    }
}

static void buttons_task(void *arg)
{
    (void) arg;
    ESP_LOGI(TAG, "Buttons task started (A=%d, B=%d, C=%d)", BTN_PIN_A, BTN_PIN_B, BTN_PIN_C);
    while (1) {
        for (size_t i = 0; i < sizeof(s_buttons) / sizeof(s_buttons[0]); ++i) {
            poll_button(&s_buttons[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_PERIOD_MS));
    }
}

esp_err_t buttons_start(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_PIN_A) | (1ULL << BTN_PIN_B) | (1ULL << BTN_PIN_C),
        .mode = GPIO_MODE_INPUT,
        /* GPIO34..39 are input-only and have no internal pull-ups; M5Stack adds external ones. */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t result = xTaskCreate(buttons_task, "task_buttons", 2048, NULL, APP_TASK_PRIORITY_NORMAL, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
