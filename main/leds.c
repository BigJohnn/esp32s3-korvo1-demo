#include "leds.h"
#include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include <time.h>

#define BLINK_GPIO CONFIG_BLINK_GPIO

static led_strip_handle_t led_strip;

void blink_led(void)
{
    /* If the addressable LED is enabled */
    // if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        srand(time(NULL));
        for(int i=0;i<12;++i) {
            led_strip_set_pixel(led_strip, i, rand()%256/2, rand()%256/2, rand()%256/2);
        }
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    // } else {
    //     /* Set all LED off to clear all pixels */
    //     led_strip_clear(led_strip);
    // }
}

void configure_led(void)
{
static const char *TAG = "leds";

    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 12, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}