#include "player.h"
#include "example_config.h"
#include "bsp/esp32_s3_korvo_1.h"
#include "esp_check.h"

static esp_codec_dev_handle_t spk_codec_dev = NULL;

static const char *TAG = "player";

extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");

esp_err_t player_spk_init()
{
    spk_codec_dev = bsp_audio_codec_speaker_init();
    if (spk_codec_dev == NULL) {
        ESP_LOGE(TAG, "bsp_audio_codec_speaker_init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "bsp_audio_codec_speaker_init success");
        esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
    }
    return ESP_OK;
}

esp_err_t player_play_wav(const char* play_filename)
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

esp_err_t player_play_canon()
{
    xTaskCreate(i2s_music, "i2s_music", 4096, NULL, 5, NULL);
    return ESP_OK;
}