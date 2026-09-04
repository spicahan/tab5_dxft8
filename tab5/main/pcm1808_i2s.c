// SPDX-License-Identifier: MIT

#include "pcm1808_i2s.h"

#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#define PCM1808_RX_DMA_DESCRIPTOR_COUNT 6U
#define PCM1808_RX_DMA_FRAMES           240U
#define PCM1808_RX_BUFFER_WORDS         (PCM1808_RX_DMA_FRAMES * PCM1808_I2S_WORDS_PER_FRAME)
#define PCM1808_READ_TIMEOUT_MS         1000U
#define PCM1808_SYNC_SEARCH_FRAMES      2048U
#define PCM1808_STARTUP_DISCARD_FRAMES  12000U

static const char *TAG = "pcm1808_i2s";

static uint32_t pattern_left_word(uint16_t sequence)
{
    return UINT32_C(0x51000000) | ((uint32_t)sequence << 8);
}

static uint32_t pattern_right_word(uint16_t sequence)
{
    const uint16_t complement = (uint16_t)~sequence;
    return UINT32_C(0xAE000000) | ((uint32_t)complement << 8);
}

static bool is_pattern_frame(uint32_t left, uint32_t right, uint16_t *sequence)
{
    if ((left & UINT32_C(0xFF0000FF)) != UINT32_C(0x51000000) ||
        (right & UINT32_C(0xFF0000FF)) != UINT32_C(0xAE000000)) {
        return false;
    }

    const uint16_t candidate = (uint16_t)(left >> 8);
    if (left != pattern_left_word(candidate) ||
        right != pattern_right_word(candidate)) {
        return false;
    }

    if (sequence != NULL) {
        *sequence = candidate;
    }
    return true;
}

static esp_err_t read_required_frames(pcm1808_i2s_t *device,
                                      uint32_t *buffer,
                                      size_t requested_frames,
                                      size_t *frames_read)
{
    const esp_err_t error = pcm1808_i2s_read(device, buffer, requested_frames,
                                             frames_read,
                                             PCM1808_READ_TIMEOUT_MS);
    if (error == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG,
                 "I2S receive timeout; check common ground and BCLK/LRCK/DOUT wiring");
    }
    if (error == ESP_OK && *frames_read == 0U) {
        ESP_LOGE(TAG, "I2S receive returned no frames");
        return ESP_FAIL;
    }
    return error;
}

esp_err_t pcm1808_i2s_init(pcm1808_i2s_t *device,
                           const pcm1808_i2s_config_t *config)
{
    ESP_RETURN_ON_FALSE(device != NULL && config != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "null configuration");
    ESP_RETURN_ON_FALSE(device->rx_channel == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "device is already initialized");
    ESP_RETURN_ON_FALSE(config->sample_rate_hz > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "sample rate is zero");

    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(config->port, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = PCM1808_RX_DMA_DESCRIPTOR_COUNT;
    channel_config.dma_frame_num = PCM1808_RX_DMA_FRAMES;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel_config, NULL,
                                        &device->rx_channel),
                        TAG, "allocate I2S master RX channel");

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = config->mclk_gpio,
            .bclk = config->bclk_gpio,
            .ws = config->lrck_gpio,
            .dout = I2S_GPIO_UNUSED,
            .din = config->din_gpio,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    standard_config.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    standard_config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    standard_config.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

    const esp_err_t error =
        i2s_channel_init_std_mode(device->rx_channel, &standard_config);
    if (error != ESP_OK) {
        (void)i2s_del_channel(device->rx_channel);
        device->rx_channel = NULL;
        return error;
    }

    device->sample_rate_hz = config->sample_rate_hz;
    device->enabled = false;
    ESP_LOGI(TAG,
             "configured I2S%d master RX: MCLK=GPIO%d, BCLK=GPIO%d, LRCK=GPIO%d, DIN=GPIO%d",
             (int)config->port, (int)config->mclk_gpio, (int)config->bclk_gpio,
             (int)config->lrck_gpio, (int)config->din_gpio);
    ESP_LOGI(TAG,
             "clocks: Fs=%" PRIu32 " Hz, MCLK=%" PRIu32
             " Hz (256fs), BCLK=%" PRIu32 " Hz (64fs)",
             config->sample_rate_hz, config->sample_rate_hz * 256U,
             config->sample_rate_hz * 64U);
    return ESP_OK;
}

esp_err_t pcm1808_i2s_enable(pcm1808_i2s_t *device)
{
    ESP_RETURN_ON_FALSE(device != NULL && device->rx_channel != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "device is not initialized");
    ESP_RETURN_ON_FALSE(!device->enabled,
                        ESP_ERR_INVALID_STATE, TAG, "channel is already enabled");

    ESP_RETURN_ON_ERROR(i2s_channel_enable(device->rx_channel), TAG,
                        "enable I2S RX channel");
    device->enabled = true;
    return ESP_OK;
}

esp_err_t pcm1808_i2s_read(pcm1808_i2s_t *device,
                           uint32_t *interleaved_words,
                           size_t frame_capacity,
                           size_t *frames_read,
                           uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(device != NULL && device->rx_channel != NULL &&
                            device->enabled && interleaved_words != NULL &&
                            frames_read != NULL && frame_capacity > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "invalid read request");
    ESP_RETURN_ON_FALSE(frame_capacity <= SIZE_MAX /
                                             (PCM1808_I2S_WORDS_PER_FRAME * sizeof(uint32_t)),
                        ESP_ERR_INVALID_SIZE, TAG, "read size overflow");

    *frames_read = 0U;
    size_t bytes_read = 0U;
    const size_t requested_bytes =
        frame_capacity * PCM1808_I2S_WORDS_PER_FRAME * sizeof(uint32_t);
    const esp_err_t error = i2s_channel_read(device->rx_channel,
                                              interleaved_words,
                                              requested_bytes,
                                              &bytes_read,
                                              timeout_ms);
    if ((bytes_read % (PCM1808_I2S_WORDS_PER_FRAME * sizeof(uint32_t))) != 0U) {
        ESP_LOGE(TAG, "I2S driver returned a partial frame: %u bytes",
                 (unsigned)bytes_read);
        return ESP_ERR_INVALID_SIZE;
    }
    *frames_read = bytes_read /
                   (PCM1808_I2S_WORDS_PER_FRAME * sizeof(uint32_t));
    return error;
}

esp_err_t pcm1808_i2s_validate_mock(pcm1808_i2s_t *device,
                                    size_t frames_to_check,
                                    pcm1808_mock_test_result_t *result)
{
    ESP_RETURN_ON_FALSE(device != NULL && frames_to_check > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "invalid mock-test request");

    pcm1808_mock_test_result_t local_result = {0};
    uint32_t *buffer = malloc(PCM1808_RX_BUFFER_WORDS * sizeof(uint32_t));
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_NO_MEM, TAG,
                        "allocate I2S receive buffer");

    // Drain one complete RX DMA ring. It removes any startup samples captured
    // before the mock's preloaded pattern reached the wire.
    size_t warmup_remaining =
        PCM1808_RX_DMA_DESCRIPTOR_COUNT * PCM1808_RX_DMA_FRAMES;
    while (warmup_remaining > 0U) {
        const size_t requested = warmup_remaining < PCM1808_RX_DMA_FRAMES
                                     ? warmup_remaining
                                     : PCM1808_RX_DMA_FRAMES;
        size_t received = 0U;
        const esp_err_t error = read_required_frames(device, buffer, requested,
                                                      &received);
        if (error != ESP_OK) {
            free(buffer);
            return error;
        }
        warmup_remaining -= received;
        local_result.startup_frames_discarded += received;
    }

    bool synchronized = false;
    uint16_t expected_sequence = 0U;
    size_t searched_frames = 0U;
    uint32_t first_left = 0U;
    uint32_t first_right = 0U;
    while (local_result.frames_checked < frames_to_check) {
        const size_t remaining = frames_to_check - local_result.frames_checked;
        const size_t requested = remaining < PCM1808_RX_DMA_FRAMES
                                     ? remaining
                                     : PCM1808_RX_DMA_FRAMES;
        size_t received = 0U;
        const esp_err_t error = read_required_frames(device, buffer, requested,
                                                      &received);
        if (error != ESP_OK) {
            free(buffer);
            return error;
        }

        for (size_t frame = 0U; frame < received; ++frame) {
            const uint32_t left =
                buffer[frame * PCM1808_I2S_WORDS_PER_FRAME];
            const uint32_t right =
                buffer[frame * PCM1808_I2S_WORDS_PER_FRAME + 1U];
            uint16_t received_sequence = 0U;

            if (!synchronized) {
                if (searched_frames == 0U) {
                    first_left = left;
                    first_right = right;
                }
                ++searched_frames;
                if (!is_pattern_frame(left, right, &received_sequence)) {
                    if (searched_frames >= PCM1808_SYNC_SEARCH_FRAMES) {
                        ESP_LOGE(TAG,
                                 "mock pattern not found; first frame L=0x%08" PRIX32
                                 ", R=0x%08" PRIX32,
                                 first_left, first_right);
                        free(buffer);
                        return ESP_ERR_INVALID_RESPONSE;
                    }
                    continue;
                }
                synchronized = true;
                expected_sequence = received_sequence;
                local_result.startup_frames_discarded += searched_frames - 1U;
                ESP_LOGI(TAG, "mock pattern synchronized at sequence 0x%04X",
                         (unsigned)expected_sequence);
            }

            if (left != pattern_left_word(expected_sequence) ||
                right != pattern_right_word(expected_sequence)) {
                ESP_LOGE(TAG,
                         "pattern mismatch at frame %u: sequence 0x%04X, "
                         "L=0x%08" PRIX32 ", R=0x%08" PRIX32,
                         (unsigned)local_result.frames_checked,
                         (unsigned)expected_sequence, left, right);
                free(buffer);
                return ESP_ERR_INVALID_RESPONSE;
            }

            ++local_result.frames_checked;
            ++expected_sequence;
            if (local_result.frames_checked == frames_to_check) {
                break;
            }
        }
    }

    ESP_LOGI(TAG,
             "mock-pattern PASS: %u continuous stereo frames",
             (unsigned)local_result.frames_checked);
    ESP_LOGI(TAG,
             "word format PASS: left/right order, Philips shift, 24-bit alignment, and zero padding");

    if (result != NULL) {
        *result = local_result;
    }
    free(buffer);
    return ESP_OK;
}

esp_err_t pcm1808_i2s_capture_stats(pcm1808_i2s_t *device,
                                    size_t frames_to_capture,
                                    pcm1808_capture_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(device != NULL && stats != NULL &&
                            frames_to_capture > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "invalid capture request");

    uint32_t *buffer = malloc(PCM1808_RX_BUFFER_WORDS * sizeof(uint32_t));
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_NO_MEM, TAG,
                        "allocate I2S receive buffer");

    pcm1808_capture_stats_t local = {
        .left_min = INT32_MAX,
        .left_max = INT32_MIN,
        .right_min = INT32_MAX,
        .right_max = INT32_MIN,
    };
    int64_t left_sum = 0;
    int64_t right_sum = 0;

    // PCM1808 DOUT remains muted while its digital filters settle after SCKI
    // starts. Discard 12,000 complete frames (> the specified 8,960/Fs
    // startup interval) so a short capture cannot report a false all-zero
    // success from the reset/mute interval.
    size_t startup_remaining = PCM1808_STARTUP_DISCARD_FRAMES;
    while (startup_remaining > 0U) {
        const size_t requested = startup_remaining < PCM1808_RX_DMA_FRAMES
                                     ? startup_remaining
                                     : PCM1808_RX_DMA_FRAMES;
        size_t received = 0U;
        const esp_err_t error = read_required_frames(device, buffer, requested,
                                                      &received);
        if (error != ESP_OK) {
            free(buffer);
            return error;
        }
        startup_remaining -= received;
    }
    ESP_LOGI(TAG, "discarded %u PCM1808 startup/mute frames",
             (unsigned)PCM1808_STARTUP_DISCARD_FRAMES);

    while (local.frames_captured < frames_to_capture) {
        const size_t remaining = frames_to_capture - local.frames_captured;
        const size_t requested = remaining < PCM1808_RX_DMA_FRAMES
                                     ? remaining
                                     : PCM1808_RX_DMA_FRAMES;
        size_t received = 0U;
        const esp_err_t error = read_required_frames(device, buffer, requested,
                                                      &received);
        if (error != ESP_OK) {
            free(buffer);
            return error;
        }

        for (size_t frame = 0U; frame < received; ++frame) {
            const uint32_t left_word =
                buffer[frame * PCM1808_I2S_WORDS_PER_FRAME];
            const uint32_t right_word =
                buffer[frame * PCM1808_I2S_WORDS_PER_FRAME + 1U];
            const int32_t left = ((int32_t)left_word) >> 8;
            const int32_t right = ((int32_t)right_word) >> 8;

            if ((left_word & UINT32_C(0xFF)) != 0U) {
                ++local.nonzero_padding_words;
            }
            if ((right_word & UINT32_C(0xFF)) != 0U) {
                ++local.nonzero_padding_words;
            }
            if (left < local.left_min) {
                local.left_min = left;
            }
            if (left > local.left_max) {
                local.left_max = left;
            }
            if (right < local.right_min) {
                local.right_min = right;
            }
            if (right > local.right_max) {
                local.right_max = right;
            }
            left_sum += left;
            right_sum += right;
            ++local.frames_captured;
        }
    }

    local.left_mean = left_sum / (int64_t)local.frames_captured;
    local.right_mean = right_sum / (int64_t)local.frames_captured;
    *stats = local;

    ESP_LOGI(TAG,
             "capture complete: %u frames, L[min=%" PRId32 ", max=%" PRId32
             ", mean=%" PRId64 "], R[min=%" PRId32 ", max=%" PRId32
             ", mean=%" PRId64 "]",
             (unsigned)local.frames_captured,
             local.left_min, local.left_max, local.left_mean,
             local.right_min, local.right_max, local.right_mean);
    if (local.nonzero_padding_words != 0U) {
        ESP_LOGE(TAG, "%u sample words had nonzero low padding bytes",
                 (unsigned)local.nonzero_padding_words);
        free(buffer);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (local.left_min == local.left_max || local.right_min == local.right_max) {
        ESP_LOGE(TAG,
                 "stuck sample stream: left_constant=%s, right_constant=%s",
                 local.left_min == local.left_max ? "yes" : "no",
                 local.right_min == local.right_max ? "yes" : "no");
        free(buffer);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG,
             "digital-link sanity PASS: both channels vary and all 24-in-32 padding is zero");

    free(buffer);
    return ESP_OK;
}

esp_err_t pcm1808_i2s_deinit(pcm1808_i2s_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (device->rx_channel == NULL) {
        memset(device, 0, sizeof(*device));
        return ESP_OK;
    }

    if (device->enabled) {
        const esp_err_t error = i2s_channel_disable(device->rx_channel);
        if (error != ESP_OK) {
            // Keep the handle and state intact so the caller can retry rather
            // than losing track of a channel that may still be running.
            return error;
        }
        device->enabled = false;
    }

    const esp_err_t error = i2s_del_channel(device->rx_channel);
    if (error != ESP_OK) {
        return error;
    }
    memset(device, 0, sizeof(*device));
    return ESP_OK;
}
