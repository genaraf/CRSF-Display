#include "display/display.h"

#include "app_config.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

static const char *TAG = "display";

/* M5Stack Basic ILI9341 pinout (SPI3/VSPI shared with SD card). */
#define DISPLAY_SPI_HOST    SPI3_HOST
#define DISPLAY_PIN_CS      14
#define DISPLAY_PIN_DC      27
#define DISPLAY_PIN_RST     33
#define DISPLAY_PIN_BL      32

/* M5Stack Basic LCD is rated for ~40 MHz; start conservatively at 20 MHz
 * to avoid signal-integrity glitches on the shared SPI3 bus with SDCard. */
#define DISPLAY_PIXEL_CLOCK_HZ      (20 * 1000 * 1000)
#define DISPLAY_LVGL_TICK_PERIOD_MS 2
#define DISPLAY_LVGL_TASK_PERIOD_MS 5
#define DISPLAY_LVGL_TASK_STACK     6144
#define DISPLAY_LVGL_TASK_PRIORITY  2

/* Physical panel dimensions (native portrait with swap_xy=false).
 * LVGL software rotation converts these to the logical 320x240 landscape. */
#define DISPLAY_PANEL_W 240
#define DISPLAY_PANEL_H 320

/* Partial buffer: PANEL_W pixels/row * N lines. 240 * 40 * 2 = 19200 B per buffer. */
#define DISPLAY_DRAW_BUFFER_LINES   40
#define DISPLAY_DRAW_BUFFER_BYTES   (DISPLAY_PANEL_W * DISPLAY_DRAW_BUFFER_LINES * sizeof(uint16_t))

static esp_lcd_panel_handle_t s_panel_handle;
static esp_lcd_panel_io_handle_t s_panel_io_handle;
static lv_display_t *s_lvgl_display;
static SemaphoreHandle_t s_lvgl_mutex;

static esp_err_t init_spi_bus(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = APP_SDCARD_MOSI_GPIO,
        .miso_io_num = APP_SDCARD_MISO_GPIO,
        .sclk_io_num = APP_SDCARD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_DRAW_BUFFER_BYTES + 32,
    };

    esp_err_t err = spi_bus_initialize(DISPLAY_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI3 bus already initialized; continuing");
        return ESP_OK;
    }
    return err;
}

static esp_err_t init_backlight(void)
{
    gpio_config_t bl_gpio = {
        .pin_bit_mask = 1ULL << DISPLAY_PIN_BL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&bl_gpio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "backlight gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }
    return gpio_set_level(DISPLAY_PIN_BL, 1);
}

static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx)
{
    (void) panel_io;
    (void) edata;
    lv_display_t *disp = (lv_display_t *) user_ctx;
    if (disp != NULL) {
        lv_display_flush_ready(disp);
    }
    return false;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t) lv_display_get_user_data(disp);
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2;
    int y2 = area->y2;

    /* Convert LVGL native RGB565 (LE) to ILI9341 wire byte order (BE). */
    lv_draw_sw_rgb565_swap(px_map, lv_area_get_size(area));

    esp_err_t err = esp_lcd_panel_draw_bitmap(panel, x1, y1, x2 + 1, y2 + 1, px_map);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
        lv_display_flush_ready(disp);
    }
}

static uint32_t lvgl_tick_get_cb(void)
{
    return (uint32_t) (esp_timer_get_time() / 1000);
}

static void lvgl_task(void *arg)
{
    (void) arg;
    ESP_LOGI(TAG, "LVGL task started");
    while (1) {
        uint32_t delay_ms = DISPLAY_LVGL_TASK_PERIOD_MS;
        if (display_lock(portMAX_DELAY)) {
            delay_ms = lv_timer_handler();
            display_unlock();
        }
        if (delay_ms > 500) {
            delay_ms = 500;
        } else if (delay_ms < 5) {
            delay_ms = 5;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

bool display_lock(uint32_t timeout_ms)
{
    if (s_lvgl_mutex == NULL) {
        return false;
    }
    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(s_lvgl_mutex, ticks) == pdTRUE;
}

void display_unlock(void)
{
    if (s_lvgl_mutex != NULL) {
        xSemaphoreGiveRecursive(s_lvgl_mutex);
    }
}

esp_err_t display_init(void)
{
    esp_err_t err;

    err = init_spi_bus();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_spi_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    err = init_backlight();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_backlight failed: %s", esp_err_to_name(err));
        return err;
    }

    /* SPI panel IO (DC/CS, command/parameter widths for ILI9341). */
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = DISPLAY_PIN_DC,
        .cs_gpio_num = DISPLAY_PIN_CS,
        .pclk_hz = DISPLAY_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) DISPLAY_SPI_HOST, &io_config, &s_panel_io_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_spi failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DISPLAY_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    err = esp_lcd_new_panel_ili9341(s_panel_io_handle, &panel_config, &s_panel_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_ili9341 failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel_handle));
    /* M5Stack Basic ILI9342C is driven in native portrait mode.
     * LVGL software rotation presents a logical 320x240 landscape screen. */
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel_handle, true));

    /* LVGL initialization. */
    lv_init();
    lv_tick_set_cb(lvgl_tick_get_cb);

    /* Create with physical panel dimensions, then rotate to logical landscape. */
    s_lvgl_display = lv_display_create(DISPLAY_PANEL_W, DISPLAY_PANEL_H);
    if (s_lvgl_display == NULL) {
        ESP_LOGE(TAG, "lv_display_create failed");
        return ESP_FAIL;
    }
    lv_display_set_rotation(s_lvgl_display, LV_DISPLAY_ROTATION_90);

    void *buf1 = heap_caps_malloc(DISPLAY_DRAW_BUFFER_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    void *buf2 = heap_caps_malloc(DISPLAY_DRAW_BUFFER_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buf1 == NULL || buf2 == NULL) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers (%u bytes each)", (unsigned) DISPLAY_DRAW_BUFFER_BYTES);
        return ESP_ERR_NO_MEM;
    }

    lv_display_set_buffers(s_lvgl_display, buf1, buf2, DISPLAY_DRAW_BUFFER_BYTES, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(s_lvgl_display, lvgl_flush_cb);
    lv_display_set_user_data(s_lvgl_display, s_panel_handle);

    /* Notify LVGL when a color transfer finishes so it can swap buffers. */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(s_panel_io_handle, &cbs, s_lvgl_display));

    s_lvgl_mutex = xSemaphoreCreateRecursiveMutex();
    if (s_lvgl_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create LVGL mutex");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t result = xTaskCreate(lvgl_task, "task_lvgl", DISPLAY_LVGL_TASK_STACK, NULL, DISPLAY_LVGL_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LVGL task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Display initialized: %dx%d, draw buffer %u bytes x2",
             DISPLAY_WIDTH, DISPLAY_HEIGHT, (unsigned) DISPLAY_DRAW_BUFFER_BYTES);
    return ESP_OK;
}
