#ifndef RECORDER_H
#define RECORDER_H

#include "esp_err.h"
esp_err_t recorder_mic_init();
esp_err_t recorder_record_wav(const char* recording_filename);

#endif