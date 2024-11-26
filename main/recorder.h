#ifndef RECORDER_H
#define RECORDER_H

#include "esp_err.h"
#include "sdmmc_cmd.h"
esp_err_t recorder_mic_init();
sdmmc_card_t * mount_sdcard(void);
esp_err_t recorder_record_wav(const char* recording_filename);

#endif