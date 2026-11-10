set(main_srcs "rbgrid_app.cpp" "rbamp_bridge.c" "esp_now_hub.c" "rbevlog.c" "rbi2c_cmd.c")
idf_component_register(SRCS ${main_srcs}
                    INCLUDE_DIRS "."
                    REQUIRES rbpower
                            esp_psram   # the firmware requires PSRAM; with MINIMAL_BUILD, IDF 5.5 drops it unless asked
                            console
                            nvs_flash
                            esp_wifi
                            esp_event
                            esp_netif
                            esp_http_server
                            rbamp
                            esp_driver_i2c
                            mbedtls
                            joltwallet__littlefs

)
