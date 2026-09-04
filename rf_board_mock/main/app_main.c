// SPDX-License-Identifier: MIT

#include "esp_err.h"
#include "esp_log.h"
#include "pcm1808_emulator.h"
#include "si5351_emulator.h"

static const char *TAG = "dxft8_mock";

void app_main(void)
{
    ESP_LOGI(TAG, "Tab5 DXFT8 RF-card mock - Si5351 + PCM1808 integration");
    ESP_ERROR_CHECK(si5351_emulator_start());
    ESP_ERROR_CHECK(pcm1808_emulator_start());
    ESP_LOGI(TAG, "Ready for the Tab5 I2C and I2S host tests");
}
