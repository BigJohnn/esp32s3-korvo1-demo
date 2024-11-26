#include "recorder.h"
#include "sdkconfig.h"
#include "bsp/esp32_s3_korvo_1.h"
#include "wav_def.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"

#include "esp_system.h"
// #include "esp_heap_caps.h"
// #include "esp_system_mem_alloc.h"
// #include "driver/sdmmc_host.h"
// #include "driver/i2s_tdm.h"
// #include "driver/i2c.h"
// #include "driver/i2c.h"
// #include "es7210.h"
#include "format_wav.h"
#include "example_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <sys/unistd.h>
#include <sys/stat.h>

// #include "es8311.h"
// #include "es7210.h"
// #include "driver/i2s_tdm.h"
// #include "driver/i2c.h"



static const char *TAG = "recorder";



static esp_codec_dev_handle_t mic_codec_dev = NULL;

esp_err_t recorder_mic_init()
{
    // /* Init I2C bus to configure ES7210 and I2S bus to receive audio data from ES7210 */
    // i2s_chan_handle_t i2s_rx_chan = es7210_i2s_init();
    // /* Create ES7210 device handle and configure codec parameters */
    // es7210_codec_init();
    // /* Mount SD card, the recorded audio file will be saved into it */
    // /* Start to record wav audio */
    // esp_err_t err = recorder_record_wav(i2s_rx_chan);
    // /* Unmount SD card */
    // esp_vfs_fat_sdcard_unmount(EXAMPLE_SD_MOUNT_POINT, sdmmc_card);
    // if (err == ESP_OK) {
    //     ESP_LOGI(TAG, "Audio was successfully recorded into "EXAMPLE_RECORD_FILE_PATH
    //              ". You can now remove the SD card safely");
    // } else {
    //     ESP_LOGE(TAG, "Record failed, "EXAMPLE_RECORD_FILE_PATH" on SD card may not be playable.");
    // }
    mic_codec_dev = bsp_audio_codec_microphone_init();
    if (mic_codec_dev == NULL) {
        ESP_LOGE(TAG, "bsp_audio_codec_microphone_init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "bsp_audio_codec_microphone_init success");
    }
    return ESP_OK;
}

esp_err_t recorder_record_wav(const char* recording_filename)
{
    ESP_LOGI(TAG, "recorder_record_wav ...");
    int16_t *recording_buffer = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    assert(recording_buffer != NULL);

    // First check if file exists before creating a new file.
    struct stat st;
    if (stat(recording_filename, &st) == 0) {
        // Delete it if it exists
        unlink(recording_filename);
    }

    // Create new WAV file
    FILE *f = fopen(recording_filename, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }

    /* Write WAV file header */
    // const dumb_wav_header_t recording_header = {
    //     .bits_per_sample = 16,
    //     .data_size = RECORDING_LENGTH * BUFFER_SIZE,
    //     .num_channels = 1,
    //     .sample_rate = EXAMPLE_SAMPLE_RATE
    // };
    #define EXAMPLE_BIT_SAMPLE 16
    #define NUM_CHANNELS 2
    #define BYTE_RATE           (EXAMPLE_SAMPLE_RATE * (EXAMPLE_BIT_SAMPLE / 8)) * NUM_CHANNELS
    const float rec_time = 10; 
    uint32_t flash_rec_time = BYTE_RATE * rec_time;
    const wav_header_t wav_header =
        WAV_HEADER_PCM_DEFAULT(flash_rec_time, 16, EXAMPLE_SAMPLE_RATE, NUM_CHANNELS);
fwrite(&wav_header, sizeof(wav_header), 1, f);
    // if (fwrite((void *)&recording_header, 1, sizeof(dumb_wav_header_t), f) != sizeof(dumb_wav_header_t)) {
    //     ESP_LOGW(TAG, "Error in writing to file");
    //     // continue;
    // }
    // setvbuf(record_file, NULL, _IOFBF, BUFFER_SIZE);

    //TODO: how to get 3rd channel's data?
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = EXAMPLE_SAMPLE_RATE,
        .channel = NUM_CHANNELS,
        // .channel_mask , default is f(1111)
        .bits_per_sample = EXAMPLE_BIT_SAMPLE,
    };
    esp_codec_dev_set_in_gain(mic_codec_dev, 42.0);
    esp_codec_dev_open(mic_codec_dev, &fs);

    ESP_LOGI(TAG, "Recording start");
    size_t bytes_written = 0;
    while (bytes_written < RECORDING_LENGTH * BUFFER_SIZE) {
        /* Read data from codec and write it to SPIFFS */
        ESP_ERROR_CHECK(esp_codec_dev_read(mic_codec_dev, recording_buffer, BUFFER_SIZE));
        size_t data_written = fwrite(recording_buffer, 1, BUFFER_SIZE, f);
        bytes_written += data_written;
    }

    ESP_LOGI(TAG, "Recording stop, length: %i bytes", bytes_written);
            fclose(f);
    free(recording_buffer);
    esp_codec_dev_close(mic_codec_dev);
    return ESP_OK;
}