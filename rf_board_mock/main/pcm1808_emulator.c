// SPDX-License-Identifier: MIT

#include "pcm1808_emulator.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdatomic.h>

#include "driver/i2s_std.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define PCM1808_FRAME_WORDS          2U
#define PCM1808_PATTERN_FRAMES       256U
#define PCM1808_PATTERN_BYTES        (PCM1808_PATTERN_FRAMES * PCM1808_FRAME_WORDS * sizeof(uint32_t))
#define PCM1808_DMA_DESCRIPTOR_COUNT 6U
#define PCM1808_WRITE_TIMEOUT_MS     1000U
#define PCM1808_MCLK_WINDOW_US       1000U
#define PCM1808_MCLK_WAIT_POLL_MS    10U
#define PCM1808_MCLK_RUN_POLL_MS     10U
#define PCM1808_MCLK_WAIT_LOG_US     UINT64_C(5000000)
#define PCM1808_MCLK_PASS_LOG_US     UINT64_C(30000000)

static const char *TAG = "pcm1808_emu";

static i2s_chan_handle_t s_tx_channel;
static uint32_t s_pattern[PCM1808_PATTERN_FRAMES * PCM1808_FRAME_WORDS];
static uint16_t s_writer_block_start;
static size_t s_writer_offset;
static bool s_stream_started;
#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
static atomic_bool s_mclk_qualified = ATOMIC_VAR_INIT(false);
#endif

static uint32_t pattern_left_word(uint16_t sequence)
{
    // The 24-bit payload is 0x51SSSS; the final byte is PCM1808 padding.
    return UINT32_C(0x51000000) | ((uint32_t)sequence << 8);
}

static uint32_t pattern_right_word(uint16_t sequence)
{
    // The right 24-bit payload is the one's complement of the left payload.
    const uint16_t complement = (uint16_t)~sequence;
    return UINT32_C(0xAE000000) | ((uint32_t)complement << 8);
}

static void build_pattern(uint16_t first_sequence)
{
    for (uint32_t frame = 0; frame < PCM1808_PATTERN_FRAMES; ++frame) {
        const uint16_t sequence = (uint16_t)(first_sequence + frame);
        s_pattern[frame * PCM1808_FRAME_WORDS] = pattern_left_word(sequence);
        s_pattern[frame * PCM1808_FRAME_WORDS + 1U] = pattern_right_word(sequence);
    }
}

static esp_err_t preload_pattern(void)
{
    uint16_t next_sequence = 0U;

    // Fill every available DMA descriptor before starting. This prevents an
    // initial run of zero samples when the Tab5 begins supplying BCLK/LRCK.
    // Generate every block at its true position in the continuous sequence;
    // repeating the same data per descriptor could hide a dropped descriptor.
    while (true) {
        build_pattern(next_sequence);
        size_t bytes_loaded = 0U;
        ESP_RETURN_ON_ERROR(i2s_channel_preload_data(s_tx_channel,
                                                      s_pattern,
                                                      PCM1808_PATTERN_BYTES,
                                                      &bytes_loaded),
                            TAG, "preload I2S DMA");
        ESP_RETURN_ON_FALSE((bytes_loaded %
                             (PCM1808_FRAME_WORDS * sizeof(uint32_t))) == 0U,
                            ESP_ERR_INVALID_SIZE, TAG,
                            "preload accepted a partial I2S frame");

        s_writer_block_start = next_sequence;
        s_writer_offset = bytes_loaded;
        next_sequence = (uint16_t)(next_sequence +
                                   bytes_loaded /
                                       (PCM1808_FRAME_WORDS * sizeof(uint32_t)));
        if (bytes_loaded < PCM1808_PATTERN_BYTES) {
            break;
        }
    }
    return ESP_OK;
}

static void i2s_writer_task(void *argument)
{
    (void)argument;
    const uint8_t *pattern_bytes = (const uint8_t *)s_pattern;
#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
    static const uint8_t silence[PCM1808_PATTERN_BYTES];
#endif
    uint16_t block_start = s_writer_block_start;
    size_t offset = s_writer_offset;
#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
    bool was_qualified = true;
#endif

    while (true) {
#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
        const bool qualified = atomic_load_explicit(&s_mclk_qualified,
                                                     memory_order_relaxed);
        if (qualified != was_qualified) {
            // A real PCM1808 cannot keep producing valid samples without
            // SCKI. Emit silence after the already-buffered frames so a later
            // host test cannot pass using only BCLK/LRCK. When SCKI returns,
            // begin a fresh continuous pattern that the host can resync to.
            offset = 0U;
            if (qualified) {
                block_start = 0U;
                build_pattern(block_start);
            }
            was_qualified = qualified;
        }
        const uint8_t *write_bytes = qualified ? pattern_bytes : silence;
#else
        const uint8_t *write_bytes = pattern_bytes;
#endif

        size_t bytes_written = 0U;
        const esp_err_t error = i2s_channel_write(s_tx_channel,
                                                   write_bytes + offset,
                                                   PCM1808_PATTERN_BYTES - offset,
                                                   &bytes_written,
                                                   PCM1808_WRITE_TIMEOUT_MS);

        offset += bytes_written;
        if ((bytes_written % (PCM1808_FRAME_WORDS * sizeof(uint32_t))) != 0U) {
            ESP_LOGE(TAG, "I2S driver accepted a partial frame: %u bytes",
                     (unsigned)bytes_written);
        }

        if (offset == PCM1808_PATTERN_BYTES) {
            offset = 0U;
#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
            if (qualified) {
#endif
                block_start = (uint16_t)(block_start + PCM1808_PATTERN_FRAMES);
                build_pattern(block_start);
#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
            }
#endif
        } else if (offset > PCM1808_PATTERN_BYTES) {
            ESP_LOGE(TAG, "I2S write accounting overflow: %u bytes",
                     (unsigned)offset);
            offset = 0U;
        }

        if (error == ESP_ERR_TIMEOUT) {
            // Normal before the Tab5 is connected or while its I2S is stopped.
            continue;
        }
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static esp_err_t start_pattern_stream(void)
{
    if (s_stream_started) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx_channel), TAG,
                        "enable I2S slave TX channel");
    if (xTaskCreate(i2s_writer_task, "pcm1808_tx", 3072, NULL, 5, NULL) != pdPASS) {
        (void)i2s_channel_disable(s_tx_channel);
        return ESP_ERR_NO_MEM;
    }
    s_stream_started = true;
    ESP_LOGI(TAG, "valid PCM1808 pattern enabled after MCLK qualification");
    return ESP_OK;
}

#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
static void mclk_monitor_task(void *argument)
{
    pcnt_unit_handle_t unit = (pcnt_unit_handle_t)argument;
    const uint32_t expected_hz = CONFIG_DXFT8_MOCK_I2S_SAMPLE_RATE_HZ * 256U;
    const uint32_t tolerance_hz =
        (expected_hz * CONFIG_DXFT8_MOCK_MCLK_TOLERANCE_PERCENT) / 100U;
    bool previous_measurement_passed = false;
    int64_t next_status_log_us = 0;

    while (true) {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
        const int64_t started_us = esp_timer_get_time();
        ESP_ERROR_CHECK(pcnt_unit_start(unit));
        esp_rom_delay_us(PCM1808_MCLK_WINDOW_US);
        ESP_ERROR_CHECK(pcnt_unit_stop(unit));
        const int64_t elapsed_us = esp_timer_get_time() - started_us;

        int pulses = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &pulses));
        const uint32_t measured_hz = (elapsed_us > 0 && pulses > 0)
                                         ? (uint32_t)(((uint64_t)pulses * UINT64_C(1000000)) /
                                                      (uint64_t)elapsed_us)
                                         : 0U;
        const uint32_t error_hz = (measured_hz > expected_hz)
                                      ? measured_hz - expected_hz
                                      : expected_hz - measured_hz;
        const bool passed = measured_hz != 0U && error_hz <= tolerance_hz;
        const int64_t now_us = esp_timer_get_time();
        const bool state_changed = passed != previous_measurement_passed;

        atomic_store_explicit(&s_mclk_qualified, passed,
                              memory_order_relaxed);

        if (measured_hz == 0U) {
            if (state_changed || now_us >= next_status_log_us) {
                ESP_LOGW(TAG, "MCLK waiting: no clock on GPIO%d",
                         CONFIG_DXFT8_MOCK_I2S_MCLK_GPIO);
                next_status_log_us = now_us + PCM1808_MCLK_WAIT_LOG_US;
            }
        } else if (passed) {
            if (state_changed || now_us >= next_status_log_us) {
                ESP_LOGI(TAG,
                         "MCLK PASS: measured=%" PRIu32 " Hz, expected=%" PRIu32 " Hz",
                         measured_hz, expected_hz);
                next_status_log_us = now_us + PCM1808_MCLK_PASS_LOG_US;
            }
            if (!s_stream_started) {
                const esp_err_t error = start_pattern_stream();
                if (error != ESP_OK) {
                    ESP_LOGE(TAG, "could not start qualified I2S pattern: %s",
                             esp_err_to_name(error));
                    abort();
                }
            }
        } else if (state_changed || now_us >= next_status_log_us) {
            ESP_LOGW(TAG, "MCLK out of tolerance: measured=%" PRIu32
                          " Hz, expected=%" PRIu32 " Hz (+/-%d%%)",
                     measured_hz, expected_hz,
                     CONFIG_DXFT8_MOCK_MCLK_TOLERANCE_PERCENT);
            next_status_log_us = now_us + PCM1808_MCLK_WAIT_LOG_US;
        }

        if (s_stream_started && state_changed && !passed) {
            ESP_LOGW(TAG,
                     "valid I2S pattern revoked until MCLK qualifies again");
        }

        previous_measurement_passed = passed;
        vTaskDelay(pdMS_TO_TICKS(s_stream_started
                                    ? PCM1808_MCLK_RUN_POLL_MS
                                    : PCM1808_MCLK_WAIT_POLL_MS));
    }
}

static esp_err_t start_mclk_monitor(void)
{
    const pcnt_unit_config_t unit_config = {
        .low_limit = -1,
        .high_limit = 32767,
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &unit), TAG,
                        "allocate MCLK pulse counter");

    const pcnt_chan_config_t channel_config = {
        .edge_gpio_num = CONFIG_DXFT8_MOCK_I2S_MCLK_GPIO,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t channel = NULL;
    esp_err_t error = pcnt_new_channel(unit, &channel_config, &channel);
    if (error != ESP_OK) {
        (void)pcnt_del_unit(unit);
        return error;
    }

    error = pcnt_channel_set_edge_action(channel,
                                          PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                          PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (error == ESP_OK) {
        error = pcnt_channel_set_level_action(channel,
                                               PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                               PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    }
    if (error == ESP_OK) {
        error = pcnt_unit_enable(unit);
    }
    if (error != ESP_OK) {
        (void)pcnt_del_channel(channel);
        (void)pcnt_del_unit(unit);
        return error;
    }

    if (xTaskCreate(mclk_monitor_task, "mclk_monitor", 3072, unit, 5, NULL) != pdPASS) {
        (void)pcnt_unit_disable(unit);
        (void)pcnt_del_channel(channel);
        (void)pcnt_del_unit(unit);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "MCLK monitor: GPIO%d, expected=%" PRIu32 " Hz (+/-%d%%)",
             CONFIG_DXFT8_MOCK_I2S_MCLK_GPIO,
             CONFIG_DXFT8_MOCK_I2S_SAMPLE_RATE_HZ * 256U,
             CONFIG_DXFT8_MOCK_MCLK_TOLERANCE_PERCENT);
    return ESP_OK;
}
#endif

esp_err_t pcm1808_emulator_start(void)
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    channel_config.dma_desc_num = PCM1808_DMA_DESCRIPTOR_COUNT;
    channel_config.dma_frame_num = PCM1808_PATTERN_FRAMES;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel_config, &s_tx_channel, NULL),
                        TAG, "allocate I2S slave TX channel");

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CONFIG_DXFT8_MOCK_I2S_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_DXFT8_MOCK_I2S_BCLK_GPIO,
            .ws = CONFIG_DXFT8_MOCK_I2S_LRCK_GPIO,
            .dout = CONFIG_DXFT8_MOCK_I2S_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    standard_config.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

    esp_err_t error = i2s_channel_init_std_mode(s_tx_channel, &standard_config);
    if (error == ESP_OK) {
        error = preload_pattern();
    }
    if (error != ESP_OK) {
        (void)i2s_del_channel(s_tx_channel);
        s_tx_channel = NULL;
        return error;
    }

#if CONFIG_DXFT8_MOCK_MONITOR_MCLK
    error = start_mclk_monitor();
    if (error != ESP_OK) {
        (void)i2s_del_channel(s_tx_channel);
        s_tx_channel = NULL;
        return error;
    }
#else
    ESP_RETURN_ON_ERROR(start_pattern_stream(), TAG,
                        "start unqualified I2S pattern");
#endif

    ESP_LOGI(TAG,
             "PCM1808 I2S slave armed: BCLK=GPIO%d, LRCK=GPIO%d, DOUT=GPIO%d",
             CONFIG_DXFT8_MOCK_I2S_BCLK_GPIO,
             CONFIG_DXFT8_MOCK_I2S_LRCK_GPIO,
             CONFIG_DXFT8_MOCK_I2S_DOUT_GPIO);
    ESP_LOGI(TAG,
             "format: %d Hz, Philips I2S, stereo, 24 valid bits in 32-bit slots, 64 BCLK/frame",
             CONFIG_DXFT8_MOCK_I2S_SAMPLE_RATE_HZ);
    return ESP_OK;
}
