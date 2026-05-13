#ifndef STORAGE_H
#define STORAGE_H

#include "esp_err.h"

esp_err_t storage_load_runtime_state(void);
esp_err_t storage_save_runtime_state(void);

#endif
