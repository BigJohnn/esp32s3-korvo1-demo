idf_component_register(SRCS "player.c" "recorder.c" "leds.c" "main.c" 
PRIV_REQUIRES esp_driver_i2s esp_driver_gpio fatfs
# INCLUDE_DIRS "$ENV{IDF_PATH}/examples/peripherals/i2s/common")
INCLUDE_DIRS "d:/Codes/esp/esp-idf/examples/peripherals/i2s/common" "."
EMBED_FILES "canon.pcm")

