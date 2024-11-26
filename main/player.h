#ifndef PLAYER_H
#define PLAYER_H

#include "esp_err.h"
esp_err_t player_spk_init();
esp_err_t player_play_wav(const char* recording_filename);
esp_err_t player_play_canon();


#endif