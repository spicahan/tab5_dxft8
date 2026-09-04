// SPDX-License-Identifier: MIT

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "pcm1808_i2s.h"
#include "si5351.h"

#if CONFIG_DXFT8_TAB5_I2C_INTERNAL_PULLUPS
#define TAB5_I2C_INTERNAL_PULLUPS true
#else
#define TAB5_I2C_INTERNAL_PULLUPS false
#endif

#if CONFIG_DXFT8_SI5351_EXTERNAL_REFERENCE
#define TAB5_SI5351_REFERENCE_MODE SI5351_REFERENCE_EXTERNAL_CLOCK
#define TAB5_SI5351_REFERENCE_DESCRIPTION "external clock on XA (0 pF)"
#elif CONFIG_DXFT8_SI5351_CRYSTAL_6PF
#define TAB5_SI5351_REFERENCE_MODE SI5351_REFERENCE_CRYSTAL_6PF
#define TAB5_SI5351_REFERENCE_DESCRIPTION "passive crystal (6 pF)"
#elif CONFIG_DXFT8_SI5351_CRYSTAL_8PF
#define TAB5_SI5351_REFERENCE_MODE SI5351_REFERENCE_CRYSTAL_8PF
#define TAB5_SI5351_REFERENCE_DESCRIPTION "passive crystal (8 pF)"
#else
#define TAB5_SI5351_REFERENCE_MODE SI5351_REFERENCE_CRYSTAL_10PF
#define TAB5_SI5351_REFERENCE_DESCRIPTION "passive crystal (10 pF)"
#endif

#define TEST_I2C_TIMEOUT_MS 1000
#define TEST_RF_HZ          7074000U

static const char *TAG = "tab5_bringup";

typedef enum {
    FRONTEND_CLOCKS_OFF,
    FRONTEND_CLOCKS_RX,
    FRONTEND_CLOCKS_TX_READY,
} frontend_clock_state_t;

static esp_err_t set_frontend_clock_state(si5351_t *clock,
                                          frontend_clock_state_t state,
                                          bool verify)
{
    uint8_t enabled_outputs = 0U;
    const char *description = "OFF";

    switch (state) {
    case FRONTEND_CLOCKS_OFF:
        break;
    case FRONTEND_CLOCKS_RX:
        enabled_outputs = SI5351_OUTPUT_CLK1;
        description = "RX (CLK1 QSD on)";
        break;
    case FRONTEND_CLOCKS_TX_READY:
        // The QSD is hard-enabled on the RF board, so keep CLK1 running while
        // adding CLK0 for the PA. G47/G48 perform the actual T/R switching.
        enabled_outputs = SI5351_OUTPUT_CLK0 | SI5351_OUTPUT_CLK1;
        description = "TX-ready (CLK0 PA + CLK1 QSD on)";
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t error = si5351_set_enabled_outputs(clock,
                                                        enabled_outputs,
                                                        verify);
    if (error == ESP_OK) {
        ESP_LOGI(TAG, "front-end clock state: %s", description);
    }
    return error;
}

static void stop_on_failure(si5351_t *clock, const char *stage, esp_err_t error)
{
    ESP_LOGE(TAG, "SELF-TEST FAILED at %s: %s", stage,
             esp_err_to_name(error));
    if (clock != NULL && clock->i2c_device != NULL) {
        (void)set_frontend_clock_state(clock, FRONTEND_CLOCKS_OFF, false);
    }
    ESP_LOGE(TAG, "outputs requested OFF; reset the board to retry");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "TAB5 DXFT8 Si5351 host self-test");
    ESP_LOGI(TAG,
             "Port A: SDA=GPIO%d, SCL=GPIO%d, address=0x%02X",
             CONFIG_DXFT8_TAB5_I2C_SDA_GPIO,
             CONFIG_DXFT8_TAB5_I2C_SCL_GPIO,
             CONFIG_DXFT8_SI5351_ADDRESS);
    ESP_LOGI(TAG, "reference=%d Hz, %s",
             CONFIG_DXFT8_SI5351_REFERENCE_HZ,
             TAB5_SI5351_REFERENCE_DESCRIPTION);

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_DXFT8_TAB5_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_DXFT8_TAB5_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = TAB5_I2C_INTERNAL_PULLUPS,
        },
    };

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t error = i2c_new_master_bus(&bus_config, &bus);
    if (error != ESP_OK) {
        stop_on_failure(NULL, "I2C bus initialization", error);
    }

    si5351_t clock = {0};
    error = si5351_init(&clock, bus, CONFIG_DXFT8_SI5351_ADDRESS,
                        CONFIG_DXFT8_SI5351_REFERENCE_HZ,
                        TAB5_SI5351_REFERENCE_MODE,
                        CONFIG_DXFT8_TAB5_I2C_FREQUENCY_HZ);
    if (error != ESP_OK) {
        stop_on_failure(NULL, "Si5351 handle initialization", error);
    }

    error = si5351_probe(&clock, TEST_I2C_TIMEOUT_MS);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "address probe", error);
    }
    ESP_LOGI(TAG, "probe PASS: found Si5351 at 0x%02X",
             CONFIG_DXFT8_SI5351_ADDRESS);

    uint8_t device_status = 0;
    uint8_t interrupt_status = 0;
    error = si5351_read_register(&clock, 0U, &device_status);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "device-status read", error);
    }
    error = si5351_read_register(&clock, 1U, &interrupt_status);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "interrupt-status read", error);
    }
    ESP_LOGI(TAG, "status PASS: device=0x%02X, interrupt=0x%02X",
             device_status, interrupt_status);

    error = si5351_configure_40m_clock_plan(&clock, TEST_RF_HZ, true);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "7.074 MHz DXFT8 clock plan", error);
    }
    ESP_LOGI(TAG,
             "7.074 MHz plan PASS: RX CLK1=28.296 MHz enabled; "
             "TX CLK0=7.074 MHz configured but disabled");

#if CONFIG_DXFT8_RUN_TX_PATH_SELF_TEST
    ESP_LOGW(TAG,
             "TX CLOCK SELF-TEST ENABLED: use only with the mock or an unpowered PA");
    vTaskDelay(pdMS_TO_TICKS(50));
    error = set_frontend_clock_state(&clock, FRONTEND_CLOCKS_TX_READY, true);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "RX-to-TX clock-state switch", error);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    error = set_frontend_clock_state(&clock, FRONTEND_CLOCKS_RX, true);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "TX-to-RX clock-state switch", error);
    }
    ESP_LOGI(TAG, "RX/TX-ready/RX clock-state switching PASS");
#else
    ESP_LOGW(TAG, "TX clock self-test disabled; final state remains RX");
#endif

    ESP_LOGI(TAG, "SI5351 SELF-TEST PASS");

#if CONFIG_DXFT8_RUN_I2S_SELF_TEST
    ESP_LOGI(TAG, "starting PCM1808-compatible I2S validation");
    const pcm1808_i2s_config_t adc_config = {
        .port = I2S_NUM_1,
        .mclk_gpio = (gpio_num_t)CONFIG_DXFT8_TAB5_I2S_MCLK_GPIO,
        .bclk_gpio = (gpio_num_t)CONFIG_DXFT8_TAB5_I2S_BCLK_GPIO,
        .lrck_gpio = (gpio_num_t)CONFIG_DXFT8_TAB5_I2S_LRCK_GPIO,
        .din_gpio = (gpio_num_t)CONFIG_DXFT8_TAB5_I2S_DIN_GPIO,
        .sample_rate_hz = CONFIG_DXFT8_TAB5_I2S_SAMPLE_RATE_HZ,
    };
    pcm1808_i2s_t adc = {0};
    error = pcm1808_i2s_init(&adc, &adc_config);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "I2S master initialization", error);
    }
    error = pcm1808_i2s_enable(&adc);
    if (error != ESP_OK) {
        const esp_err_t cleanup_error = pcm1808_i2s_deinit(&adc);
        if (cleanup_error != ESP_OK) {
            ESP_LOGE(TAG, "I2S cleanup also failed: %s",
                     esp_err_to_name(cleanup_error));
        }
        stop_on_failure(&clock, "I2S clock/RX start", error);
    }

#if CONFIG_DXFT8_I2S_MOCK_PATTERN_TEST
    pcm1808_mock_test_result_t mock_result = {0};
    error = pcm1808_i2s_validate_mock(&adc,
                                      CONFIG_DXFT8_I2S_TEST_FRAMES,
                                      &mock_result);
    if (error != ESP_OK) {
        const esp_err_t cleanup_error = pcm1808_i2s_deinit(&adc);
        if (cleanup_error != ESP_OK) {
            ESP_LOGE(TAG, "I2S cleanup also failed: %s",
                     esp_err_to_name(cleanup_error));
        }
        stop_on_failure(&clock, "I2S mock-pattern validation", error);
    }
    ESP_LOGI(TAG,
             "I2S SELF-TEST PASS: %u frames checked after %u startup frames",
             (unsigned)mock_result.frames_checked,
             (unsigned)mock_result.startup_frames_discarded);
#else
    pcm1808_capture_stats_t capture_stats = {0};
    error = pcm1808_i2s_capture_stats(&adc,
                                      CONFIG_DXFT8_I2S_TEST_FRAMES,
                                      &capture_stats);
    if (error != ESP_OK) {
        const esp_err_t cleanup_error = pcm1808_i2s_deinit(&adc);
        if (cleanup_error != ESP_OK) {
            ESP_LOGE(TAG, "I2S cleanup also failed: %s",
                     esp_err_to_name(cleanup_error));
        }
        stop_on_failure(&clock, "PCM1808 audio capture", error);
    }
    ESP_LOGI(TAG, "I2S CAPTURE SANITY PASS: %u PCM1808 frames received",
             (unsigned)capture_stats.frames_captured);
#endif

    error = pcm1808_i2s_deinit(&adc);
    if (error != ESP_OK) {
        stop_on_failure(&clock, "I2S shutdown", error);
    }
#else
    ESP_LOGW(TAG, "I2S self-test disabled; only the Si5351 test was run");
#endif

    ESP_LOGI(TAG, "ALL ENABLED SELF-TESTS PASS; final RF clock state remains RX");
}
