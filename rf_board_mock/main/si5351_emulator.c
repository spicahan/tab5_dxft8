// SPDX-License-Identifier: MIT

#include "si5351_emulator.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_slave.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define SI5351_REGISTER_COUNT        256U
#define SI5351_LOG_DATA_BYTES        16U
#define SI5351_LOG_QUEUE_DEPTH       128U
#define SI5351_RESPONSE_STACK_BYTES  3072U
#define SI5351_RESPONSE_PRIORITY     12U
#define SI5351_LOG_STACK_BYTES       4096U
#define SI5351_LOG_PRIORITY          5U
#define SI5351_I2C_SEND_BUFFER_BYTES 32U
#define SI5351_I2C_RECV_BUFFER_BYTES 320U
#define SI5351_I2C_WRITE_TIMEOUT_MS  1000

#define SI5351_REG_OUTPUT_ENABLE     3U
#define SI5351_REG_PLL_INPUT         15U
#define SI5351_REG_CLK0_CONTROL      16U
#define SI5351_REG_CLK1_CONTROL      17U
#define SI5351_REG_CLK2_CONTROL      18U
#define SI5351_REG_CLK7_CONTROL      23U
#define SI5351_REG_CLK3_0_DISABLE    24U
#define SI5351_REG_CLK7_4_DISABLE    25U
#define SI5351_REG_PLLA_PARAMETERS   26U
#define SI5351_REG_PLLB_PARAMETERS   34U
#define SI5351_REG_MS0_PARAMETERS    42U
#define SI5351_REG_MS1_PARAMETERS    50U
#define SI5351_REG_PLL_RESET         177U
#define SI5351_REG_XTAL_LOAD         183U

#define SI5351_PARAMETER_BYTES       8U
#define SI5351_STATUS_LOL_A          BIT(5)
#define SI5351_STATUS_LOL_B          BIT(6)
#define SI5351_CLK_POWER_DOWN        BIT(7)
#define SI5351_CLK_INTEGER_MODE      BIT(6)
#define SI5351_CLK_PLLB_SOURCE       BIT(5)
#define SI5351_CLK_SOURCE_MASK       (BIT(3) | BIT(2))
#define SI5351_CLK_SOURCE_MULTISYNTH SI5351_CLK_SOURCE_MASK
#define SI5351_PLLA_RESET            BIT(5)
#define SI5351_PLLB_RESET            BIT(7)
#define SI5351_UNUSED_OUTPUT_MASK    0xFCU
#define SI5351_EXPECTED_RF_HZ        7074000ULL
#define SI5351_EXPECTED_QSD_HZ       (SI5351_EXPECTED_RF_HZ * 4ULL)
#define SI5351_EXPECTED_PLL_HZ       (SI5351_EXPECTED_RF_HZ * 112ULL)
#define SI5351_EXPECTED_CLK0_CONTROL 0x4FU
#define SI5351_EXPECTED_CLK1_CONTROL 0x6FU

#define SI5351_OUTPUTS_OFF           0xFFU
#define SI5351_RX_CLOCKS             0xFDU
#define SI5351_TX_READY_CLOCKS       0xFCU
#define SI5351_TX_ONLY_CLOCK         0xFEU

#if CONFIG_DXFT8_MOCK_SI5351_EXTERNAL_REFERENCE
#define SI5351_EXPECTED_XTAL_LOAD    0x12U
#define SI5351_REFERENCE_MODE_NAME   "active external reference on XA"
#elif CONFIG_DXFT8_MOCK_SI5351_CRYSTAL_6PF
#define SI5351_EXPECTED_XTAL_LOAD    0x52U
#define SI5351_REFERENCE_MODE_NAME   "6 pF passive crystal"
#elif CONFIG_DXFT8_MOCK_SI5351_CRYSTAL_8PF
#define SI5351_EXPECTED_XTAL_LOAD    0x92U
#define SI5351_REFERENCE_MODE_NAME   "8 pF passive crystal"
#elif CONFIG_DXFT8_MOCK_SI5351_CRYSTAL_10PF
#define SI5351_EXPECTED_XTAL_LOAD    0xD2U
#define SI5351_REFERENCE_MODE_NAME   "10 pF passive crystal"
#else
#error "Select one Si5351 reference mode"
#endif

#if CONFIG_DXFT8_MOCK_I2C_INTERNAL_PULLUPS
#define SI5351_INTERNAL_PULLUPS true
#else
#define SI5351_INTERNAL_PULLUPS false
#endif

typedef enum {
    SI5351_LOG_POINTER,
    SI5351_LOG_WRITE,
    SI5351_LOG_READ,
} si5351_log_kind_t;

typedef struct {
    uint32_t p1;
    uint32_t p2;
    uint32_t p3;
    uint32_t integer;
    uint32_t numerator;
    uint32_t denominator;
} si5351_ratio_t;

typedef struct {
    bool report;
    bool outputs_off_seen;
    bool configuration_changed_while_enabled;
    uint8_t pll_reset_mask_seen;
    uint8_t output_enable;
    uint8_t pll_input;
    uint8_t clock_control[8];
    uint8_t disabled_state[2];
    uint8_t pll[2][SI5351_PARAMETER_BYTES];
    uint8_t multisynth[2][SI5351_PARAMETER_BYTES];
    uint8_t xtal_load;
} si5351_clock_snapshot_t;

typedef struct {
    si5351_log_kind_t kind;
    uint8_t start_register;
    uint8_t value;
    uint16_t length;
    bool truncated;
    uint8_t data[SI5351_LOG_DATA_BYTES];
    si5351_clock_snapshot_t clock;
} si5351_log_event_t;

typedef struct {
    i2c_slave_dev_handle_t i2c_handle;
    QueueHandle_t log_queue;
    TaskHandle_t response_task;
    uint8_t registers[SI5351_REGISTER_COUNT];
    uint8_t register_pointer;
    volatile uint8_t pending_read_register;
    volatile uint8_t pending_read_value;
    volatile uint32_t dropped_log_events;
    bool outputs_off_seen;
    bool configuration_changed_while_enabled;
    uint8_t pll_reset_mask_seen;
} si5351_emulator_t;

static const char *TAG = "si5351_emu";
static si5351_emulator_t s_emulator;

static bool si5351_decode_pll_hz(
    const uint8_t bytes[SI5351_PARAMETER_BYTES], uint64_t *frequency_hz,
    si5351_ratio_t *ratio);
static void si5351_update_pll_lock_status(si5351_emulator_t *emulator,
                                          uint8_t reset_mask);

static void si5351_queue_log_from_isr(si5351_emulator_t *emulator,
                                      const si5351_log_event_t *event,
                                      BaseType_t *higher_priority_task_woken)
{
#if CONFIG_DXFT8_MOCK_LOG_TRANSACTIONS
    if (xQueueSendFromISR(emulator->log_queue, event,
                          higher_priority_task_woken) != pdTRUE) {
        emulator->dropped_log_events++;
    }
#else
    (void)emulator;
    (void)event;
    (void)higher_priority_task_woken;
#endif
}

static bool si5351_is_synth_configuration_register(uint8_t reg)
{
    return reg == SI5351_REG_PLL_INPUT ||
           (reg >= SI5351_REG_CLK0_CONTROL && reg <= 25U) ||
           (reg >= SI5351_REG_PLLA_PARAMETERS && reg <= 65U) ||
           reg == SI5351_REG_XTAL_LOAD;
}

static void si5351_write_shadow_register(si5351_emulator_t *emulator,
                                         uint8_t reg, uint8_t value)
{
    if (reg == 0U) {
        // Device status is read-only.
        return;
    }
    if (reg == 1U) {
        // Sticky interrupt status is write-zero-to-clear.
        emulator->registers[1] &= value;
        return;
    }
    if (reg == SI5351_REG_PLL_RESET) {
        // PLL reset bits are strobes and read back cleared.
        const uint8_t reset_mask =
            value & (SI5351_PLLA_RESET | SI5351_PLLB_RESET);
        if (reset_mask != 0U) {
            emulator->pll_reset_mask_seen |= reset_mask;
            si5351_update_pll_lock_status(emulator, reset_mask);
        }
        emulator->registers[SI5351_REG_PLL_RESET] = 0;
        return;
    }

    if (reg == SI5351_REG_OUTPUT_ENABLE) {
        emulator->registers[reg] = value;
        if (value == SI5351_OUTPUTS_OFF) {
            emulator->outputs_off_seen = true;
        }
        return;
    }

    if (si5351_is_synth_configuration_register(reg)) {
        if (emulator->registers[SI5351_REG_OUTPUT_ENABLE] !=
            SI5351_OUTPUTS_OFF) {
            emulator->configuration_changed_while_enabled = true;
        }
        if ((reg >= SI5351_REG_PLLA_PARAMETERS && reg <= 65U) ||
            reg == SI5351_REG_PLL_INPUT || reg == SI5351_REG_XTAL_LOAD) {
            emulator->pll_reset_mask_seen = 0U;
        }
        if (reg >= SI5351_REG_PLLA_PARAMETERS &&
            reg < SI5351_REG_PLLB_PARAMETERS) {
            emulator->registers[0] |= SI5351_STATUS_LOL_A;
        } else if (reg >= SI5351_REG_PLLB_PARAMETERS &&
                   reg < SI5351_REG_MS0_PARAMETERS) {
            emulator->registers[0] |= SI5351_STATUS_LOL_B;
        } else if (reg == SI5351_REG_PLL_INPUT ||
                   reg == SI5351_REG_XTAL_LOAD) {
            emulator->registers[0] |=
                SI5351_STATUS_LOL_A | SI5351_STATUS_LOL_B;
        }
    }
    emulator->registers[reg] = value;
}

static bool si5351_write_contains_register(uint8_t start_register,
                                           size_t length, uint8_t target)
{
    for (size_t index = 0; index < length; ++index) {
        if ((uint8_t)(start_register + index) == target) {
            return true;
        }
    }
    return false;
}

static void si5351_capture_clock_snapshot(const si5351_emulator_t *emulator,
                                          si5351_clock_snapshot_t *snapshot)
{
    snapshot->report = true;
    snapshot->outputs_off_seen = emulator->outputs_off_seen;
    snapshot->configuration_changed_while_enabled =
        emulator->configuration_changed_while_enabled;
    snapshot->pll_reset_mask_seen = emulator->pll_reset_mask_seen;
    snapshot->output_enable = emulator->registers[SI5351_REG_OUTPUT_ENABLE];
    snapshot->pll_input = emulator->registers[SI5351_REG_PLL_INPUT];
    memcpy(snapshot->clock_control,
           &emulator->registers[SI5351_REG_CLK0_CONTROL],
           sizeof(snapshot->clock_control));
    snapshot->disabled_state[0] =
        emulator->registers[SI5351_REG_CLK3_0_DISABLE];
    snapshot->disabled_state[1] =
        emulator->registers[SI5351_REG_CLK7_4_DISABLE];
    memcpy(snapshot->pll[0],
           &emulator->registers[SI5351_REG_PLLA_PARAMETERS],
           SI5351_PARAMETER_BYTES);
    memcpy(snapshot->pll[1],
           &emulator->registers[SI5351_REG_PLLB_PARAMETERS],
           SI5351_PARAMETER_BYTES);
    memcpy(snapshot->multisynth[0],
           &emulator->registers[SI5351_REG_MS0_PARAMETERS],
           SI5351_PARAMETER_BYTES);
    memcpy(snapshot->multisynth[1],
           &emulator->registers[SI5351_REG_MS1_PARAMETERS],
           SI5351_PARAMETER_BYTES);
    snapshot->xtal_load = emulator->registers[SI5351_REG_XTAL_LOAD];
}

static bool si5351_receive_callback(i2c_slave_dev_handle_t handle,
                                    const i2c_slave_rx_done_event_data_t *event_data,
                                    void *user_data)
{
    (void)handle;
    si5351_emulator_t *emulator = (si5351_emulator_t *)user_data;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (event_data->length == 0) {
        return false;
    }

    const uint8_t start_register = event_data->buffer[0];
    emulator->register_pointer = start_register;

    for (size_t index = 1; index < event_data->length; ++index) {
        const uint8_t reg = emulator->register_pointer;
        si5351_write_shadow_register(emulator, reg, event_data->buffer[index]);
        emulator->register_pointer++;
    }

    si5351_log_event_t log_event = {
        .kind = event_data->length == 1 ? SI5351_LOG_POINTER : SI5351_LOG_WRITE,
        .start_register = start_register,
        .length = (uint16_t)(event_data->length - 1U),
    };
    size_t log_length = log_event.length;
    if (log_length > sizeof(log_event.data)) {
        log_length = sizeof(log_event.data);
        log_event.truncated = true;
    }
    if (log_length > 0) {
        memcpy(log_event.data, &event_data->buffer[1], log_length);
    }
    if (si5351_write_contains_register(start_register, log_event.length,
                                       SI5351_REG_OUTPUT_ENABLE)) {
        si5351_capture_clock_snapshot(emulator, &log_event.clock);
    }
    si5351_queue_log_from_isr(emulator, &log_event,
                              &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static bool si5351_request_callback(i2c_slave_dev_handle_t handle,
                                    const i2c_slave_request_event_data_t *event_data,
                                    void *user_data)
{
    (void)handle;
    (void)event_data;
    si5351_emulator_t *emulator = (si5351_emulator_t *)user_data;

    const uint8_t reg = emulator->register_pointer;
    emulator->pending_read_register = reg;
    emulator->pending_read_value = emulator->registers[reg];
    emulator->register_pointer++;

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(emulator->response_task,
                           &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static void si5351_reset_shadow_registers(si5351_emulator_t *emulator)
{
    memset(emulator->registers, 0, sizeof(emulator->registers));

    // Status reports initialization complete, PLLs locked and reference present.
    emulator->registers[0] = 0x00;
    emulator->registers[1] = 0x00;

    // Documented reset value: 10 pF crystal load plus reserved bit pattern.
    emulator->registers[183] = 0xD2;
    // The hardware's output-enable register resets enabled. Keep that reset
    // behavior so the host test must prove that it establishes a safe state.
    emulator->registers[SI5351_REG_OUTPUT_ENABLE] = 0x00;
    emulator->register_pointer = 0;
    emulator->outputs_off_seen = false;
    emulator->configuration_changed_while_enabled = false;
    emulator->pll_reset_mask_seen = 0U;
}

static bool si5351_decode_ratio(const uint8_t bytes[SI5351_PARAMETER_BYTES],
                                si5351_ratio_t *ratio)
{
    ratio->p3 = ((uint32_t)(bytes[5] & 0xF0U) << 12U) |
                ((uint32_t)bytes[0] << 8U) | bytes[1];
    ratio->p1 = ((uint32_t)(bytes[2] & 0x03U) << 16U) |
                ((uint32_t)bytes[3] << 8U) | bytes[4];
    ratio->p2 = ((uint32_t)(bytes[5] & 0x0FU) << 16U) |
                ((uint32_t)bytes[6] << 8U) | bytes[7];

    if (ratio->p3 == 0U) {
        return false;
    }

    const uint32_t scaled_integer = ratio->p1 + 512U;
    const uint32_t fractional_floor = scaled_integer & 0x7FU;
    const uint64_t raw_numerator =
        (uint64_t)ratio->p2 + (uint64_t)ratio->p3 * fractional_floor;
    if ((raw_numerator & 0x7FU) != 0U) {
        return false;
    }

    ratio->integer = scaled_integer >> 7U;
    ratio->numerator = (uint32_t)(raw_numerator >> 7U);
    ratio->denominator = ratio->p3;
    if (ratio->numerator >= ratio->denominator) {
        return false;
    }
    return true;
}

static bool si5351_ratio_is_integer(const si5351_ratio_t *ratio)
{
    return ratio->numerator == 0U;
}

static bool si5351_decode_pll_hz(
    const uint8_t bytes[SI5351_PARAMETER_BYTES], uint64_t *frequency_hz,
    si5351_ratio_t *ratio)
{
    if (!si5351_decode_ratio(bytes, ratio)) {
        return false;
    }
    if (ratio->integer < 15U || ratio->integer > 90U) {
        return false;
    }

    const uint64_t total_numerator =
        (uint64_t)ratio->integer * ratio->denominator + ratio->numerator;
    const uint64_t scaled_frequency =
        (uint64_t)CONFIG_DXFT8_MOCK_SI5351_REFERENCE_HZ * total_numerator;
    *frequency_hz =
        (scaled_frequency + ratio->denominator / 2U) / ratio->denominator;
    return *frequency_hz >= 600000000ULL && *frequency_hz <= 900000000ULL;
}

static void si5351_update_pll_lock_status(si5351_emulator_t *emulator,
                                          uint8_t reset_mask)
{
    const bool reference_valid =
        emulator->registers[SI5351_REG_PLL_INPUT] == 0x00U &&
        emulator->registers[SI5351_REG_XTAL_LOAD] ==
            SI5351_EXPECTED_XTAL_LOAD;
    si5351_ratio_t ratio;
    uint64_t frequency_hz = 0;

    if ((reset_mask & SI5351_PLLA_RESET) != 0U) {
        const bool valid = reference_valid &&
            si5351_decode_pll_hz(
                &emulator->registers[SI5351_REG_PLLA_PARAMETERS],
                &frequency_hz, &ratio);
        if (valid) {
            emulator->registers[0] &= (uint8_t)~SI5351_STATUS_LOL_A;
        } else {
            emulator->registers[0] |= SI5351_STATUS_LOL_A;
        }
    }

    if ((reset_mask & SI5351_PLLB_RESET) != 0U) {
        const bool valid = reference_valid &&
            si5351_decode_pll_hz(
                &emulator->registers[SI5351_REG_PLLB_PARAMETERS],
                &frequency_hz, &ratio);
        if (valid) {
            emulator->registers[0] &= (uint8_t)~SI5351_STATUS_LOL_B;
        } else {
            emulator->registers[0] |= SI5351_STATUS_LOL_B;
        }
    }
}

static bool si5351_decode_output_hz(
    const uint8_t pll_bytes[SI5351_PARAMETER_BYTES],
    const uint8_t ms_bytes[SI5351_PARAMETER_BYTES], uint64_t *pll_hz,
    uint64_t *output_hz, si5351_ratio_t *ms_ratio)
{
    si5351_ratio_t pll_ratio;
    if (!si5351_decode_pll_hz(pll_bytes, pll_hz, &pll_ratio) ||
        !si5351_decode_ratio(ms_bytes, ms_ratio)) {
        return false;
    }

    const uint32_t r_divider = 1U << ((ms_bytes[2] >> 4U) & 0x07U);
    const bool divide_by_four = (ms_bytes[2] & 0x0CU) == 0x0CU;
    if (divide_by_four) {
        if (ms_ratio->p1 != 0U || ms_ratio->p2 != 0U ||
            ms_ratio->p3 != 1U) {
            return false;
        }
        *output_hz = (*pll_hz + (2U * r_divider)) / (4U * r_divider);
        return true;
    }

    if (ms_ratio->integer < 8U || ms_ratio->integer > 2048U) {
        return false;
    }
    const uint64_t divider_numerator =
        ((uint64_t)ms_ratio->integer * ms_ratio->denominator +
         ms_ratio->numerator) * r_divider;
    const uint64_t frequency_numerator = *pll_hz * ms_ratio->denominator;
    *output_hz = (frequency_numerator + divider_numerator / 2U) /
                 divider_numerator;
    return true;
}

static bool si5351_log_clock(const si5351_clock_snapshot_t *clock,
                             unsigned clock_index, unsigned expected_pll_index,
                             uint64_t expected_output_hz,
                             uint32_t expected_divider)
{
    const uint8_t control = clock->clock_control[clock_index];
    const unsigned pll_index =
        (control & SI5351_CLK_PLLB_SOURCE) != 0U ? 1U : 0U;
    uint64_t pll_hz = 0;
    uint64_t output_hz = 0;
    si5351_ratio_t ms_ratio;

    if ((control & SI5351_CLK_POWER_DOWN) != 0U) {
        ESP_LOGE(TAG, "CLK%u is enabled in reg3 but its driver is powered down",
                 clock_index);
        return false;
    }
    if (pll_index != expected_pll_index) {
        ESP_LOGE(TAG, "CLK%u uses PLL%c, expected PLL%c",
                 clock_index, pll_index == 0U ? 'A' : 'B',
                 expected_pll_index == 0U ? 'A' : 'B');
        return false;
    }
    if ((control & SI5351_CLK_SOURCE_MASK) !=
        SI5351_CLK_SOURCE_MULTISYNTH) {
        ESP_LOGE(TAG,
                 "CLK%u source is not MultiSynth%u (control=0x%02X)",
                 clock_index, clock_index, control);
        return false;
    }
    if (!si5351_decode_output_hz(clock->pll[pll_index],
                                 clock->multisynth[clock_index], &pll_hz,
                                 &output_hz, &ms_ratio)) {
        ESP_LOGE(TAG, "CLK%u has invalid PLL/MultiSynth parameters",
                 clock_index);
        return false;
    }

    const bool integer_mode_requested =
        (control & SI5351_CLK_INTEGER_MODE) != 0U;
    const bool valid_integer_mode =
        si5351_ratio_is_integer(&ms_ratio) &&
        (ms_ratio.integer & 1U) == 0U;
    if (integer_mode_requested && !valid_integer_mode) {
        ESP_LOGE(TAG,
                 "CLK%u forces integer mode for a non-even divider",
                 clock_index);
        return false;
    }
    if (pll_hz != SI5351_EXPECTED_PLL_HZ ||
        output_hz != expected_output_hz ||
        ms_ratio.integer != expected_divider || ms_ratio.numerator != 0U) {
        ESP_LOGE(TAG,
                 "CLK%u frequency plan mismatch: PLL=%" PRIu64
                 " Hz, output=%" PRIu64 ", divider=%" PRIu32
                 "+%" PRIu32 "/%" PRIu32,
                 clock_index, pll_hz, output_hz, ms_ratio.integer,
                 ms_ratio.numerator, ms_ratio.denominator);
        return false;
    }

    ESP_LOGI(TAG,
             "CLK%u valid: PLL%c=%" PRIu64 " Hz, output=%" PRIu64
             " Hz, MS=%" PRIu32 "+%" PRIu32 "/%" PRIu32,
             clock_index, pll_index == 0U ? 'A' : 'B', pll_hz, output_hz,
             ms_ratio.integer, ms_ratio.numerator, ms_ratio.denominator);
    return true;
}

static void si5351_log_clock_snapshot(const si5351_clock_snapshot_t *clock)
{
    if (!clock->report || clock->output_enable == SI5351_OUTPUTS_OFF) {
        if (clock->report) {
            ESP_LOGI(TAG, "all clock outputs disabled (safe state)");
        }
        return;
    }

    bool valid = true;
    if (!clock->outputs_off_seen) {
        ESP_LOGE(TAG, "unsafe order: outputs were not disabled before setup");
        valid = false;
    }
    if (clock->configuration_changed_while_enabled) {
        ESP_LOGE(TAG, "unsafe order: synthesizer changed while an output was enabled");
        valid = false;
    }
    const bool clk0_enabled = (clock->output_enable & BIT(0)) == 0U;
    const bool clk1_enabled = (clock->output_enable & BIT(1)) == 0U;
    uint8_t required_pll_reset_mask = 0U;
    if (clk0_enabled) {
        required_pll_reset_mask |= SI5351_PLLA_RESET;
    }
    if (clk1_enabled) {
        required_pll_reset_mask |= SI5351_PLLB_RESET;
    }
    if ((clock->pll_reset_mask_seen & required_pll_reset_mask) !=
        required_pll_reset_mask) {
        ESP_LOGE(TAG,
                 "unsafe order: output enabled without required PLL reset "
                 "(seen=0x%02X, required=0x%02X)",
                 clock->pll_reset_mask_seen, required_pll_reset_mask);
        valid = false;
    }
    if (clock->pll_input != 0x00U) {
        ESP_LOGE(TAG, "PLL input register is 0x%02X, expected XA/XO path 0x00",
                 clock->pll_input);
        valid = false;
    }
    if (clock->xtal_load != SI5351_EXPECTED_XTAL_LOAD) {
        ESP_LOGE(TAG,
                 "reference-load register is 0x%02X, expected 0x%02X (%s)",
                 clock->xtal_load, SI5351_EXPECTED_XTAL_LOAD,
                 SI5351_REFERENCE_MODE_NAME);
        valid = false;
    }
    if ((clock->output_enable & SI5351_UNUSED_OUTPUT_MASK) !=
        SI5351_UNUSED_OUTPUT_MASK) {
        ESP_LOGE(TAG, "unused CLK2..CLK7 enabled (register 3=0x%02X)",
                 clock->output_enable);
        valid = false;
    }
    if (clock->clock_control[0] != SI5351_EXPECTED_CLK0_CONTROL ||
        clock->clock_control[1] != SI5351_EXPECTED_CLK1_CONTROL) {
        ESP_LOGE(TAG,
                 "clock controls are CLK0=0x%02X CLK1=0x%02X; expected "
                 "0x%02X/0x%02X",
                 clock->clock_control[0], clock->clock_control[1],
                 SI5351_EXPECTED_CLK0_CONTROL,
                 SI5351_EXPECTED_CLK1_CONTROL);
        valid = false;
    }
    for (unsigned index = 2U; index < 8U; ++index) {
        if ((clock->clock_control[index] & SI5351_CLK_POWER_DOWN) == 0U) {
            ESP_LOGE(TAG, "unused CLK%u driver is not powered down", index);
            valid = false;
        }
    }
    if (clock->disabled_state[0] != 0x00U ||
        clock->disabled_state[1] != 0x00U) {
        ESP_LOGE(TAG,
                 "disabled-output state is not low (regs24/25=0x%02X/0x%02X)",
                 clock->disabled_state[0], clock->disabled_state[1]);
        valid = false;
    }

    switch (clock->output_enable) {
    case SI5351_RX_CLOCKS:
        ESP_LOGI(TAG, "front-end clock state RX: CLK1 QSD enabled");
        break;
    case SI5351_TX_READY_CLOCKS:
        ESP_LOGI(TAG,
                 "front-end clock state TX-ready: CLK0 PA + CLK1 QSD enabled");
        break;
    case SI5351_TX_ONLY_CLOCK:
        ESP_LOGE(TAG,
                 "invalid TX-only clock state: CLK1/QSD must keep running");
        valid = false;
        break;
    default:
        ESP_LOGE(TAG, "unsupported output-enable state 0x%02X",
                 clock->output_enable);
        valid = false;
        break;
    }

    if (clk0_enabled) {
        valid = si5351_log_clock(clock, 0U, 0U,
                                 SI5351_EXPECTED_RF_HZ, 112U) && valid;
    }
    if (clk1_enabled) {
        valid = si5351_log_clock(clock, 1U, 1U,
                                 SI5351_EXPECTED_QSD_HZ, 28U) && valid;
    }

    if (valid) {
        ESP_LOGI(TAG,
                 "PASS: Si5351 programming order, clock plan and state are valid");
    }
}

static void si5351_response_task(void *argument)
{
    si5351_emulator_t *emulator = (si5351_emulator_t *)argument;

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const uint8_t reg = emulator->pending_read_register;
        const uint8_t value = emulator->pending_read_value;
        uint32_t written = 0;
        const esp_err_t error = i2c_slave_write(
            emulator->i2c_handle, &value, sizeof(value), &written,
            SI5351_I2C_WRITE_TIMEOUT_MS);

        if (error != ESP_OK) {
            ESP_LOGE(TAG, "read response failed at register 0x%02X: %s",
                     reg, esp_err_to_name(error));
            continue;
        }
        if (written != sizeof(value)) {
            ESP_LOGW(TAG, "read response was not staged (register 0x%02X)",
                     reg);
            continue;
        }

#if CONFIG_DXFT8_MOCK_LOG_TRANSACTIONS
        const si5351_log_event_t log_event = {
            .kind = SI5351_LOG_READ,
            .start_register = reg,
            .value = value,
        };
        if (xQueueSend(emulator->log_queue, &log_event, 0) != pdTRUE) {
            emulator->dropped_log_events++;
        }
#endif
    }
}

#if CONFIG_DXFT8_MOCK_LOG_TRANSACTIONS
static void si5351_log_task(void *argument)
{
    si5351_emulator_t *emulator = (si5351_emulator_t *)argument;
    uint32_t last_reported_drop_count = 0;

    while (true) {
        si5351_log_event_t event;
        if (xQueueReceive(emulator->log_queue, &event,
                          pdMS_TO_TICKS(1000)) == pdTRUE) {
            switch (event.kind) {
            case SI5351_LOG_POINTER:
                ESP_LOGI(TAG, "pointer <- 0x%02X", event.start_register);
                break;
            case SI5351_LOG_WRITE:
                ESP_LOGI(TAG, "write start=0x%02X, bytes=%u%s",
                         event.start_register, (unsigned)event.length,
                         event.truncated ? " (log truncated)" : "");
                ESP_LOG_BUFFER_HEX_LEVEL(
                    TAG, event.data,
                    event.length < sizeof(event.data) ? event.length
                                                       : sizeof(event.data),
                    ESP_LOG_INFO);
                si5351_log_clock_snapshot(&event.clock);
                break;
            case SI5351_LOG_READ:
                ESP_LOGI(TAG, "read  reg=0x%02X -> 0x%02X",
                         event.start_register, event.value);
                break;
            }
        }

        const uint32_t dropped = emulator->dropped_log_events;
        if (dropped != last_reported_drop_count) {
            ESP_LOGW(TAG, "transaction logs dropped=%" PRIu32, dropped);
            last_reported_drop_count = dropped;
        }
    }
}
#endif

esp_err_t si5351_emulator_start(void)
{
    si5351_emulator_t *emulator = &s_emulator;
    si5351_reset_shadow_registers(emulator);

    emulator->log_queue = xQueueCreate(SI5351_LOG_QUEUE_DEPTH,
                                        sizeof(si5351_log_event_t));
    ESP_RETURN_ON_FALSE(emulator->log_queue != NULL, ESP_ERR_NO_MEM, TAG,
                        "failed to allocate log queue");

    const BaseType_t response_task_created = xTaskCreate(
        si5351_response_task, "si5351_response", SI5351_RESPONSE_STACK_BYTES,
        emulator, SI5351_RESPONSE_PRIORITY, &emulator->response_task);
    ESP_RETURN_ON_FALSE(response_task_created == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create response task");

#if CONFIG_DXFT8_MOCK_LOG_TRANSACTIONS
    const BaseType_t log_task_created = xTaskCreate(
        si5351_log_task, "si5351_log", SI5351_LOG_STACK_BYTES, emulator,
        SI5351_LOG_PRIORITY, NULL);
    ESP_RETURN_ON_FALSE(log_task_created == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create log task");
#endif

    const i2c_slave_config_t slave_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_DXFT8_MOCK_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_DXFT8_MOCK_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth = SI5351_I2C_SEND_BUFFER_BYTES,
        .receive_buf_depth = SI5351_I2C_RECV_BUFFER_BYTES,
        .slave_addr = CONFIG_DXFT8_MOCK_SI5351_ADDRESS,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .intr_priority = 0,
        .flags = {
            .enable_internal_pullup = SI5351_INTERNAL_PULLUPS,
        },
    };

    ESP_RETURN_ON_ERROR(i2c_new_slave_device(&slave_config,
                                              &emulator->i2c_handle),
                        TAG, "failed to initialize I2C slave");

    const i2c_slave_event_callbacks_t callbacks = {
        .on_receive = si5351_receive_callback,
        .on_request = si5351_request_callback,
    };
    ESP_RETURN_ON_ERROR(i2c_slave_register_event_callbacks(
                            emulator->i2c_handle, &callbacks, emulator),
                        TAG, "failed to register I2C callbacks");

    ESP_LOGI(TAG,
             "ready: address=0x%02X (7-bit), SDA=GPIO%d, SCL=GPIO%d",
             CONFIG_DXFT8_MOCK_SI5351_ADDRESS,
             CONFIG_DXFT8_MOCK_I2C_SDA_GPIO,
             CONFIG_DXFT8_MOCK_I2C_SCL_GPIO);
    ESP_LOGI(TAG, "frequency decoder reference=%d Hz (%s), register 183=0x%02X",
             CONFIG_DXFT8_MOCK_SI5351_REFERENCE_HZ,
             SI5351_REFERENCE_MODE_NAME, SI5351_EXPECTED_XTAL_LOAD);
    ESP_LOGI(TAG, "status register models PLL lock after reset strobes");
    ESP_LOGW(TAG, "current milestone supports one response byte per read request");
    return ESP_OK;
}
