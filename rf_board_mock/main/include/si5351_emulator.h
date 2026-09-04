// SPDX-License-Identifier: MIT

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the Si5351 I2C mock.
 *
 * The device acknowledges the configured seven-bit address, records writes
 * in a 256-byte shadow register file, and returns one shadow-register byte
 * for each read request.
 */
esp_err_t si5351_emulator_start(void);

#ifdef __cplusplus
}
#endif
