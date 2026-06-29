set(srcs
    rctank.c
    rctank_motor.c
    rctank_servo.c
    rctank_led.c
    rctank_dfplayer.c
    rctank_storage.c
)

idf_component_register(SRCS "rctank_dfplayer.c" "rctank_servo.c" "rctank_motor.c" "${srcs}"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer log freertos
)
