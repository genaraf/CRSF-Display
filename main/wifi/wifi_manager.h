#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t wifi_manager_start(void);
void wifi_manager_set_web_ui_ready(bool ready);

#endif
