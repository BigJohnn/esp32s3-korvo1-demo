/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "sdkconfig.h"
#include "wav_def.h"

/* Example configurations */
#define EXAMPLE_RECV_BUF_SIZE   (2400)
#define EXAMPLE_SAMPLE_RATE     (16000)
#define EXAMPLE_MCLK_MULTIPLE   (384) // If not using 24-bit data width, 256 should be enough
#define EXAMPLE_MCLK_FREQ_HZ    (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)
#define EXAMPLE_VOICE_VOLUME    CONFIG_EXAMPLE_VOICE_VOLUME
#if CONFIG_EXAMPLE_MODE_ECHO
#define EXAMPLE_MIC_GAIN        CONFIG_EXAMPLE_MIC_GAIN
#endif

#define DEFAULT_VOLUME  (100)

#define EXAMPLE_SD_MOUNT_POINT     "/sdcard"

#define BUFFER_SIZE     (1024)
#define RECORDING_LENGTH (160)

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
#include "bsp/esp-bsp.h"
#define I2C_NUM BSP_I2C_NUM
