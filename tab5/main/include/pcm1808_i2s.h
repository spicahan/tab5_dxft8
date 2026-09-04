// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** PCM1808-compatible wire format: stereo, 24 significant bits in 32-bit slots. */
#define PCM1808_I2S_WORDS_PER_FRAME 2U

typedef struct {
    i2s_port_t port;
    gpio_num_t mclk_gpio;
    gpio_num_t bclk_gpio;
    gpio_num_t lrck_gpio;
    gpio_num_t din_gpio;
    uint32_t sample_rate_hz;
} pcm1808_i2s_config_t;

typedef struct {
    i2s_chan_handle_t rx_channel;
    uint32_t sample_rate_hz;
    bool enabled;
} pcm1808_i2s_t;

typedef struct {
    size_t frames_checked;
    size_t startup_frames_discarded;
} pcm1808_mock_test_result_t;

typedef struct {
    size_t frames_captured;
    int32_t left_min;
    int32_t left_max;
    int32_t right_min;
    int32_t right_max;
    int64_t left_mean;
    int64_t right_mean;
    size_t nonzero_padding_words;
} pcm1808_capture_stats_t;

/**
 * Allocate/configure master RX. On ESP32-P4, MCLK may become active here;
 * BCLK, LRCK, and RX DMA remain stopped until pcm1808_i2s_enable().
 */
esp_err_t pcm1808_i2s_init(pcm1808_i2s_t *device,
                           const pcm1808_i2s_config_t *config);

/** Start BCLK, LRCK, and RX DMA. */
esp_err_t pcm1808_i2s_enable(pcm1808_i2s_t *device);

/** Read up to frame_capacity interleaved 32-bit left/right frames. */
esp_err_t pcm1808_i2s_read(pcm1808_i2s_t *device,
                           uint32_t *interleaved_words,
                           size_t frame_capacity,
                           size_t *frames_read,
                           uint32_t timeout_ms);

/**
 * Validate the deterministic stream emitted by rf_board_mock.
 *
 * This test intentionally fails against a real PCM1808 audio stream. Use
 * pcm1808_i2s_capture_stats() for first-board bring-up with the actual ADC.
 */
esp_err_t pcm1808_i2s_validate_mock(pcm1808_i2s_t *device,
                                    size_t frames_to_check,
                                    pcm1808_mock_test_result_t *result);

/**
 * Capture ordinary PCM1808 audio, summarize it, and reject stuck streams or
 * nonzero post-LSB padding. This is a digital-link sanity test, not an analog
 * performance measurement.
 */
esp_err_t pcm1808_i2s_capture_stats(pcm1808_i2s_t *device,
                                    size_t frames_to_capture,
                                    pcm1808_capture_stats_t *stats);

/** Stop clocks/RX and release the I2S channel. Safe on a zeroed handle. */
esp_err_t pcm1808_i2s_deinit(pcm1808_i2s_t *device);

#ifdef __cplusplus
}
#endif
