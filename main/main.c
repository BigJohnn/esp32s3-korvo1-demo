/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "leds.h"
#include "sdkconfig.h"
#include <string.h>

#include "esp_system.h"
#include "esp_check.h"
#include "example_config.h"

#include "recorder.h"
#include "player.h"

static const char *TAG = "korvo1-main";

void app_main(void)
{
    ESP_ERROR_CHECK(bsp_spiffs_mount());

    recorder_mic_init();
    player_spk_init();

    const char recording_filename[] = BSP_SPIFFS_MOUNT_POINT"/recording.wav";
    recorder_record_wav(recording_filename);
    player_play_wav(recording_filename);
    player_play_canon();

    /* Configure the peripheral according to the LED type */
    configure_led();

    while (1) {
        blink_led();
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
