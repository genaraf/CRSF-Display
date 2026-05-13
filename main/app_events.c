#include "app_events.h"

#include <string.h>

#include "freertos/queue.h"

static QueueHandle_t s_app_events_queue;

esp_err_t app_events_init(size_t queue_length)
{
    if (s_app_events_queue != NULL) {
        return ESP_OK;
    }

    s_app_events_queue = xQueueCreate(queue_length, sizeof(app_event_t));
    if (s_app_events_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t app_events_post(const app_event_t *event, TickType_t timeout_ticks)
{
    if ((s_app_events_queue == NULL) || (event == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    app_event_t event_copy;
    memcpy(&event_copy, event, sizeof(event_copy));

    if (xQueueSend(s_app_events_queue, &event_copy, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

bool app_events_receive(app_event_t *event, TickType_t timeout_ticks)
{
    if ((s_app_events_queue == NULL) || (event == NULL)) {
        return false;
    }

    return xQueueReceive(s_app_events_queue, event, timeout_ticks) == pdTRUE;
}
