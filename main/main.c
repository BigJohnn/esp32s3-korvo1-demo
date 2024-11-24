/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "leds.h"
#include "sdkconfig.h"

#include <string.h>
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "driver/i2s_tdm.h"
// #include "driver/i2c.h"
#include "driver/i2c.h"
#include "es7210.h"
#include "format_wav.h"

/******************************************* */
#include "driver/i2s_std.h"
#include "esp_system.h"
#include "esp_check.h"
#include "es8311.h"
#include "example_config.h"

#include "bsp/esp32_s3_korvo_1.h"

static const char *TAG = "korvo1-main";
#define BUFFER_SIZE     (1024)
#define DEFAULT_VOLUME  (100)
#define RECORDING_LENGTH (160)
// static const char err_reason[][30] = {"input param is invalid",
//                                       "operation timeout"
//                                      };
static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;

static esp_codec_dev_handle_t spk_codec_dev = NULL;
static esp_codec_dev_handle_t mic_codec_dev = NULL;

/* Import music file as buffer */
#if CONFIG_EXAMPLE_MODE_MUSIC
extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");
#endif
// static esp_err_t es8311_codec_init(void)
// {
//     /* Initialize I2C peripheral */
// #if !defined(CONFIG_EXAMPLE_BSP)
//     const i2c_config_t es_i2c_cfg = {
//         .sda_io_num = I2C_SDA_IO,
//         .scl_io_num = I2C_SCL_IO,
//         .mode = I2C_MODE_MASTER,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = 100000,
//     };
//     ESP_RETURN_ON_ERROR(i2c_param_config(I2C_NUM, &es_i2c_cfg), TAG, "config i2c failed");
//     ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0), TAG, "install i2c driver failed");
// #else
//     ESP_ERROR_CHECK(bsp_i2c_init());
// #endif

//     /* Initialize es8311 codec */
//     es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
//     ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");
//     const es8311_clock_config_t es_clk = {
//         .mclk_inverted = false,
//         .sclk_inverted = false,
//         .mclk_from_mclk_pin = true,
//         .mclk_frequency = EXAMPLE_MCLK_FREQ_HZ,
//         .sample_frequency = EXAMPLE_SAMPLE_RATE
//     };

//     ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16)); //这句报错
//     ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE, EXAMPLE_SAMPLE_RATE), TAG, "set es8311 sample frequency failed");
//     ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG, "set es8311 volume failed");
//     ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");
// #if CONFIG_EXAMPLE_MODE_ECHO
//     ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN), TAG, "set es8311 microphone gain failed");
// #endif
//     return ESP_OK;
// }

#if CONFIG_EXAMPLE_MODE_MUSIC
static void i2s_music(void *args) {
    esp_err_t ret = ESP_OK;
    size_t bytes_written = 0;
    uint8_t *data_ptr = (uint8_t *)music_pcm_start;

    // audio_codec_data_if_t* spk = bsp_audio_get_codec_itf_spk();

    // esp_codec_dev_sample_info_t fs = {
    //                 .sample_rate = EXAMPLE_SAMPLE_RATE,
    //                 .channel = 1,
    //                 .bits_per_sample = 16,
    //             };
    // spk->open(spk, &fs, sizeof(fs));
    // if(!spk->is_open(spk)) {
    //     ESP_LOGE(TAG, "[music] spk open failed, %s", err_reason[ret == ESP_FAIL]);
    //         vTaskDelete(NULL); // Terminate the task on error
    //         return;
    // }
    // spk->enable(spk, ESP_CODEC_DEV_TYPE_OUT, true);
    // spk->set_fmt(spk, ESP_CODEC_DEV_TYPE_OUT, &fs);
    // esp_codec_dev_handle_t spk_codec_dev = bsp_audio_codec_speaker_init();

    esp_codec_dev_sample_info_t fs = {
                    .sample_rate = EXAMPLE_SAMPLE_RATE,
                    .channel = 1,
                    .bits_per_sample = 16,
                };
                esp_codec_dev_open(spk_codec_dev, &fs);

    while (data_ptr < music_pcm_end) {
        int step = BUFFER_SIZE > music_pcm_end - data_ptr?BUFFER_SIZE:music_pcm_end - data_ptr;
        // ret = spk->write(spk, data_ptr, );
         esp_codec_dev_write(spk_codec_dev, data_ptr, step);
        // spk_codec_dev
        // if (ret < music_pcm_end - data_ptr) {
        //     ESP_LOGE(TAG, "[music] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
        //     vTaskDelete(NULL); // Terminate the task on error
        //     return;
        // }

        bytes_written += step;
        data_ptr += step;

        ESP_LOGI(TAG, "[music] i2s music played, %d bytes written.", bytes_written);

        // Optional: Add a delay between writes if needed
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "[music] Music playback completed.");
    vTaskDelete(NULL);
}

#else
static void i2s_echo(void *args)
{
    int *mic_data = malloc(EXAMPLE_RECV_BUF_SIZE);
    if (!mic_data) {
        ESP_LOGE(TAG, "[echo] No memory for read data buffer");
        abort();
    }
    esp_err_t ret = ESP_OK;
    size_t bytes_read = 0;
    size_t bytes_write = 0;
    ESP_LOGI(TAG, "[echo] Echo start");

    while (1) {
        memset(mic_data, 0, EXAMPLE_RECV_BUF_SIZE);
        /* Read sample data from mic */
        ret = i2s_channel_read(rx_handle, mic_data, EXAMPLE_RECV_BUF_SIZE, &bytes_read, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[echo] i2s read failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
            abort();
        }
        /* Write sample data to earphone */
        ret = i2s_channel_write(tx_handle, mic_data, EXAMPLE_RECV_BUF_SIZE, &bytes_write, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[echo] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
            abort();
        }
        if (bytes_read != bytes_write) {
            ESP_LOGW(TAG, "[echo] %d bytes read but only %d bytes are written", bytes_read, bytes_write);
        }
    }
    vTaskDelete(NULL);
}
#endif
/******************************************* */

#if CONFIG_IDF_TARGET_ESP32S3 // ESP32-S3-Korvo-1 pin out
/* I2C port and GPIOs */
#define EXAMPLE_I2C_NUM            (0)
#define EXAMPLE_I2C_SDA_IO         (1)
#define EXAMPLE_I2C_SCL_IO         (2)

/* I2S port and GPIOs */
#define EXAMPLE_I2S_NUM            (0)
#define EXAMPLE_I2S_MCK_IO         (20)
#define EXAMPLE_I2S_BCK_IO         (10)
#define EXAMPLE_I2S_WS_IO          (9)
#define EXAMPLE_I2S_DI_IO          (11)

/* SD card SPI GPIOs */
#define EXAMPLE_SD_SPI_CLK_IO      (18)
#define EXAMPLE_SD_SPI_MOSI_IO     (17)
#define EXAMPLE_SD_SPI_MISO_IO     (16)
#define EXAMPLE_SD_SPI_CS_IO       (15)

#elif CONFIG_IDF_TARGET_ESP32P4

#define EXAMPLE_I2C_NUM            (0)
#define EXAMPLE_I2C_SDA_IO         (3)
#define EXAMPLE_I2C_SCL_IO         (2)

/* I2S port and GPIOs */
#define EXAMPLE_I2S_NUM            (0)
#define EXAMPLE_I2S_MCK_IO         (4)
#define EXAMPLE_I2S_BCK_IO         (5)
#define EXAMPLE_I2S_WS_IO          (6)
#define EXAMPLE_I2S_DI_IO          (7)

/* SD card SPI GPIOs */
#define EXAMPLE_SD_SPI_CLK_IO      (18)
#define EXAMPLE_SD_SPI_MOSI_IO     (19)
#define EXAMPLE_SD_SPI_MISO_IO     (14)
#define EXAMPLE_SD_SPI_CS_IO       (17)
#else
#define EXAMPLE_I2C_NUM            (0)
#define EXAMPLE_I2C_SDA_IO         (3)
#define EXAMPLE_I2C_SCL_IO         (2)

/* I2S port and GPIOs */
#define EXAMPLE_I2S_NUM            (0)
#define EXAMPLE_I2S_MCK_IO         (0)
#define EXAMPLE_I2S_BCK_IO         (1)
#define EXAMPLE_I2S_WS_IO          (10)
#define EXAMPLE_I2S_DI_IO          (8)

/* SD card SPI GPIOs */
#define EXAMPLE_SD_SPI_CLK_IO      (5)
#define EXAMPLE_SD_SPI_MOSI_IO     (7)
#define EXAMPLE_SD_SPI_MISO_IO     (6)
#define EXAMPLE_SD_SPI_CS_IO       (4)
#endif

/* I2S configurations */
#define EXAMPLE_I2S_TDM_FORMAT     (ES7210_I2S_FMT_I2S)
#define EXAMPLE_I2S_CHAN_NUM       (4)
#define EXAMPLE_I2S_SAMPLE_RATE    (48000)
#define EXAMPLE_I2S_MCLK_MULTIPLE  (I2S_MCLK_MULTIPLE_256)
#define EXAMPLE_I2S_SAMPLE_BITS    (I2S_DATA_BIT_WIDTH_16BIT)
#define EXAMPLE_I2S_TDM_SLOT_MASK  (I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3)

/* ES7210 configurations */
#define EXAMPLE_ES7210_I2C_ADDR    (0x40)
#define EXAMPLE_ES7210_I2C_CLK     (100000)
#define EXAMPLE_ES7210_MIC_GAIN    (ES7210_MIC_GAIN_30DB)
#define EXAMPLE_ES7210_MIC_BIAS    (ES7210_MIC_BIAS_2V87)
#define EXAMPLE_ES7210_ADC_VOLUME  (0)

/* SD card & recording configurations */
#define EXAMPLE_RECORD_TIME_SEC    (10)
#define EXAMPLE_SD_MOUNT_POINT     "/sdcard"
#define EXAMPLE_RECORD_FILE_PATH   "/RECORD.WAV"


static i2s_chan_handle_t es7210_i2s_init(void)
{
    i2s_chan_handle_t i2s_rx_chan = NULL;
    ESP_LOGI(TAG, "Create I2S receive channel");
    i2s_chan_config_t i2s_rx_conf = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_rx_conf, NULL, &i2s_rx_chan));

    ESP_LOGI(TAG, "Configure I2S receive channel to TDM mode");
    i2s_tdm_config_t i2s_tdm_rx_conf = {
#if EXAMPLE_I2S_FORMAT == ES7210_I2S_FMT_I2S
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(EXAMPLE_I2S_SAMPLE_BITS, I2S_SLOT_MODE_STEREO, EXAMPLE_I2S_TDM_SLOT_MASK),
#elif EXAMPLE_I2S_FORMAT == ES7210_I2S_FMT_LJ
        .slot_cfg = I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(EXAMPLE_I2S_SAMPLE_BITS, I2S_SLOT_MODE_STEREO, EXAMPLE_I2S_TDM_SLOT_MASK),
#elif EXAMPLE_I2S_FORMAT == ES7210_I2S_FMT_DSP_A
        .slot_cfg = I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(EXAMPLE_I2S_SAMPLE_BITS, I2S_SLOT_MODE_STEREO, EXAMPLE_I2S_TDM_SLOT_MASK),
#elif EXAMPLE_I2S_FORMAT == ES7210_I2S_FMT_DSP_B
        .slot_cfg = I2S_TDM_PCM_LONG_SLOT_DEFAULT_CONFIG(EXAMPLE_I2S_SAMPLE_BITS, I2S_SLOT_MODE_STEREO, EXAMPLE_I2S_TDM_SLOT_MASK),
#endif
        .clk_cfg  = {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .sample_rate_hz = EXAMPLE_I2S_SAMPLE_RATE,
            .mclk_multiple = EXAMPLE_I2S_MCLK_MULTIPLE
        },
        .gpio_cfg = {
            .mclk = EXAMPLE_I2S_MCK_IO,
            .bclk = EXAMPLE_I2S_BCK_IO,
            .ws   = EXAMPLE_I2S_WS_IO,
            .dout = -1, // ES7210 only has ADC capability
            .din  = EXAMPLE_I2S_DI_IO
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_rx_chan, &i2s_tdm_rx_conf));

    return i2s_rx_chan;
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

// static void es7210_codec_init(void)
// {
//     ESP_LOGI(TAG, "Init I2C used to configure ES7210");
//     i2c_config_t i2c_conf = {
//         .sda_io_num = EXAMPLE_I2C_SDA_IO,
//         .scl_io_num = EXAMPLE_I2C_SCL_IO,
//         .mode = I2C_MODE_MASTER,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = EXAMPLE_ES7210_I2C_CLK,
//     };
//     ESP_ERROR_CHECK(i2c_param_config(EXAMPLE_I2C_NUM, &i2c_conf));
//     ESP_ERROR_CHECK(i2c_driver_install(EXAMPLE_I2C_NUM, i2c_conf.mode, 0, 0, 0));

//     /* Create ES7210 device handle */
//     es7210_dev_handle_t es7210_handle = NULL;
//     es7210_i2c_config_t es7210_i2c_conf = {
//         .i2c_port = EXAMPLE_I2C_NUM,
//         .i2c_addr = EXAMPLE_ES7210_I2C_ADDR
//     };
//     ESP_ERROR_CHECK(es7210_new_codec(&es7210_i2c_conf, &es7210_handle));

//     ESP_LOGI(TAG, "Configure ES7210 codec parameters");
//     es7210_codec_config_t codec_conf = {
//         .i2s_format = EXAMPLE_I2S_TDM_FORMAT,
//         .mclk_ratio = EXAMPLE_I2S_MCLK_MULTIPLE,
//         .sample_rate_hz = EXAMPLE_I2S_SAMPLE_RATE,
//         .bit_width = (es7210_i2s_bits_t)EXAMPLE_I2S_SAMPLE_BITS,
//         .mic_bias = EXAMPLE_ES7210_MIC_BIAS,
//         .mic_gain = EXAMPLE_ES7210_MIC_GAIN,
//         .flags.tdm_enable = true
//     };
//     ESP_ERROR_CHECK(es7210_config_codec(es7210_handle, &codec_conf));
//     ESP_ERROR_CHECK(es7210_config_volume(es7210_handle, EXAMPLE_ES7210_ADC_VOLUME));
// }

// Very simple WAV header, ignores most fields
typedef struct __attribute__((packed))
{
    uint8_t ignore_0[22];
    uint16_t num_channels;
    uint32_t sample_rate;
    uint8_t ignore_1[6];
    uint16_t bits_per_sample;
    uint8_t ignore_2[4];
    uint32_t data_size;
    uint8_t data[];
} dumb_wav_header_t;

static esp_err_t play_wav(const char* play_filename)
{
    int16_t *wav_bytes = malloc(BUFFER_SIZE);
                assert(wav_bytes != NULL);

                /* Open WAV file */
                ESP_LOGI(TAG, "Playing file %s", play_filename);
                FILE *play_file = fopen(play_filename, "rb");
                if (play_file == NULL) {
                    ESP_LOGW(TAG, "%s file does not exist!", play_filename);
                    // break;
                }

                /* Read WAV header file */
                dumb_wav_header_t wav_header;
                if (fread((void *)&wav_header, 1, sizeof(wav_header), play_file) != sizeof(wav_header)) {
                    ESP_LOGW(TAG, "Error in reading file");
                    // break;
                }
                ESP_LOGI(TAG, "Number of channels: %" PRIu16 "", wav_header.num_channels);
                ESP_LOGI(TAG, "Bits per sample: %" PRIu16 "", wav_header.bits_per_sample);
                ESP_LOGI(TAG, "Sample rate: %" PRIu32 "", wav_header.sample_rate);
                ESP_LOGI(TAG, "Data size: %" PRIu32 "", wav_header.data_size);

                esp_codec_dev_sample_info_t fs = {
                    .sample_rate = wav_header.sample_rate,
                    .channel = wav_header.num_channels,
                    .bits_per_sample = wav_header.bits_per_sample,
                };
                esp_codec_dev_open(spk_codec_dev, &fs);

                uint32_t bytes_send_to_i2s = 0;
                while (bytes_send_to_i2s < wav_header.data_size) {
                    /* Get data from SPIFFS and send it to codec */
                    size_t bytes_read_from_spiffs = fread(wav_bytes, 1, BUFFER_SIZE, play_file);
                    esp_codec_dev_write(spk_codec_dev, wav_bytes, bytes_read_from_spiffs);
                    bytes_send_to_i2s += bytes_read_from_spiffs;
                }
                fclose(play_file);
                free(wav_bytes);
                // esp_codec_dev_close(spk_codec_dev);
    return ESP_OK;
}

static esp_err_t record_wav()
{
    int16_t *recording_buffer = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    assert(recording_buffer != NULL);

    const char recording_filename[] = BSP_SPIFFS_MOUNT_POINT"/recording.wav";
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

    play_wav(recording_filename);
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(bsp_spiffs_mount());

    printf("i2s es8311 codec example start\n-----------------------------\n");
    mic_codec_dev = bsp_audio_codec_microphone_init();
    if (mic_codec_dev == NULL) {
        ESP_LOGE(TAG, "bsp_audio_codec_microphone_init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "bsp_audio_codec_microphone_init success");
    }
    spk_codec_dev = bsp_audio_codec_speaker_init();
    if (spk_codec_dev == NULL) {
        ESP_LOGE(TAG, "bsp_audio_codec_speaker_init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "bsp_audio_codec_speaker_init success");
        esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
    }

    record_wav();
#if CONFIG_EXAMPLE_MODE_MUSIC
    /* Play a piece of music in music mode */
    xTaskCreate(i2s_music, "i2s_music", 4096, NULL, 5, NULL);
#else
    /* Echo the sound from MIC in echo mode */
    xTaskCreate(i2s_echo, "i2s_echo", 8192, NULL, 5, NULL);
#endif

    // /* Init I2C bus to configure ES7210 and I2S bus to receive audio data from ES7210 */
    // i2s_chan_handle_t i2s_rx_chan = es7210_i2s_init();
    // /* Create ES7210 device handle and configure codec parameters */
    // es7210_codec_init();
    // /* Mount SD card, the recorded audio file will be saved into it */
    // sdmmc_card_t *sdmmc_card = mount_sdcard();
    // /* Start to record wav audio */
    // esp_err_t err = record_wav(i2s_rx_chan);
    // /* Unmount SD card */
    // esp_vfs_fat_sdcard_unmount(EXAMPLE_SD_MOUNT_POINT, sdmmc_card);
    // if (err == ESP_OK) {
    //     ESP_LOGI(TAG, "Audio was successfully recorded into "EXAMPLE_RECORD_FILE_PATH
    //              ". You can now remove the SD card safely");
    // } else {
    //     ESP_LOGE(TAG, "Record failed, "EXAMPLE_RECORD_FILE_PATH" on SD card may not be playable.");
    // }

    /* Configure the peripheral according to the LED type */
    configure_led();

    while (1) {
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        // s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
