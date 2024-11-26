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
    // sdmmc_card_t *sdmmc_card = mount_sdcard();
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

sdmmc_card_t * mount_sdcard(void)
{
    sdmmc_host_t sdmmc_host = SDSPI_HOST_DEFAULT();
    sdmmc_card_t *sdmmc_card = NULL;

    ESP_LOGI(TAG, "Initializing SPI bus for SD card");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = EXAMPLE_SD_SPI_MOSI_IO,
        .miso_io_num = EXAMPLE_SD_SPI_MISO_IO,
        .sclk_io_num = EXAMPLE_SD_SPI_CLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(sdmmc_host.slot, &bus_cfg, SPI_DMA_CH_AUTO));

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = EXAMPLE_SD_SPI_CS_IO;
    slot_config.host_id = sdmmc_host.slot;

    ESP_LOGI(TAG, "Mounting SD card");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 2,
        .allocation_unit_size = 8 * 1024
    };

    esp_err_t ret;
    while (1) {
        ret = esp_vfs_fat_sdspi_mount(EXAMPLE_SD_MOUNT_POINT, &sdmmc_host, &slot_config, &mount_config, &sdmmc_card);
        if (ret == ESP_OK) {
            break;
        } else if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Card size: %lluMB, speed: %dMHz",
             (((uint64_t)sdmmc_card->csd.capacity) * sdmmc_card->csd.sector_size) >> 20,
             sdmmc_card->max_freq_khz / 1000);

    return sdmmc_card;
}

esp_err_t recorder_record_wav(const char* recording_filename)
{
    int16_t *recording_buffer = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    assert(recording_buffer != NULL);

    /* Open file for recording */
    FILE *record_file = fopen(recording_filename, "wb");
    assert(record_file != NULL);

    /* Write WAV file header */
    const dumb_wav_header_t recording_header = {
        .bits_per_sample = 16,
        .data_size = RECORDING_LENGTH * BUFFER_SIZE,
        .num_channels = 1,
        .sample_rate = EXAMPLE_SAMPLE_RATE
    };
    if (fwrite((void *)&recording_header, 1, sizeof(dumb_wav_header_t), record_file) != sizeof(dumb_wav_header_t)) {
        ESP_LOGW(TAG, "Error in writing to file");
        // continue;
    }
    setvbuf(record_file, NULL, _IOFBF, BUFFER_SIZE);

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = EXAMPLE_SAMPLE_RATE,
        .channel = 1,
        .bits_per_sample = 16,
    };
    esp_codec_dev_set_in_gain(mic_codec_dev, 42.0);
    esp_codec_dev_open(mic_codec_dev, &fs);

    ESP_LOGI(TAG, "Recording start");
    size_t bytes_written_to_spiffs = 0;
    while (bytes_written_to_spiffs < RECORDING_LENGTH * BUFFER_SIZE) {
        /* Read data from codec and write it to SPIFFS */
        ESP_ERROR_CHECK(esp_codec_dev_read(mic_codec_dev, recording_buffer, BUFFER_SIZE));
        size_t data_written = fwrite(recording_buffer, 1, BUFFER_SIZE, record_file);
        bytes_written_to_spiffs += data_written;
    }

    ESP_LOGI(TAG, "Recording stop, length: %i bytes", bytes_written_to_spiffs);
            fclose(record_file);
    free(recording_buffer);
    esp_codec_dev_close(mic_codec_dev);

    return ESP_OK;
}