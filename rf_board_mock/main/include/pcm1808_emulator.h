// SPDX-License-Identifier: MIT

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the PCM1808-compatible I2S slave transmitter and MCLK monitor.
 *
 * The emulator accepts BCLK/LRCK from the Tab5 and emits a deterministic,
 * PCM1808-shaped stereo stream: Philips I2S, 24 significant bits in 32-bit
 * slots, 64 BCLK per frame. The low eight bits of each slot are zero.
 */
esp_err_t pcm1808_emulator_start(void);

#ifdef __cplusplus
}
#endif
