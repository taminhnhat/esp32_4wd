idf_component_register(SRCS "app/main.c"
                            "modules/battery/battery.c"
                            "modules/motor/motor_control.c"
                       PRIV_REQUIRES "esp_driver_rmt" "esp_driver_pcnt" "esp_timer" "esp_driver_i2c"
                       INCLUDE_DIRS "."
                                    "config"
                                    "modules/battery"
                                    "modules/motor")
