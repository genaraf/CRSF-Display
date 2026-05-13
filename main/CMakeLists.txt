idf_component_register(
    SRCS
        "CRSF-Display.c"
        "app_events.c"
        "app_state.c"
        "crsf/crsf_rx.c"
        "crsf/crsf_tx.c"
        "gps/gps_input.c"
        "simulator/simulator.c"
        "telemetry/telemetry_manager.c"
        "ui/ui.c"
        "web/web_ui.c"
        "wifi/wifi_manager.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_http_server
        esp_netif
        esp_wifi
        esp_event
        json
        nvs_flash
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
