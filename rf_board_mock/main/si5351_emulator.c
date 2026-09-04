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
#define SI5351_LOG_QUEUE_DEPTH       16U
#define SI5351_RESPONSE_STACK_BYTES  3072U
#define SI5351_RESPONSE_PRIORITY     12U
#define SI5351_LOG_STACK_BYTES       4096U
#define SI5351_LOG_PRIORITY          5U
#define SI5351_I2C_SEND_BUFFER_BYTES 32U
#define SI5351_I2C_RECV_BUFFER_BYTES 320U
#define SI5351_I2C_WRITE_TIMEOUT_MS  1000

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
    si5351_log_kind_t kind;
    uint8_t start_register;
    uint8_t value;
    uint16_t length;
    bool truncated;
    uint8_t data[SI5351_LOG_DATA_BYTES];
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
} si5351_emulator_t;

static const char *TAG = "si5351_emu";
static si5351_emulator_t s_emulator;

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
    if (reg == 177U) {
        // PLL reset bits are strobes and read back cleared.
        emulator->registers[177] = 0;
        return;
    }
    emulator->registers[reg] = value;
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

    // Status reports initialization complete, PLLs locked and crystal present.
    emulator->registers[0] = 0x00;
    emulator->registers[1] = 0x00;

    // Documented reset value: 10 pF crystal load plus reserved bit pattern.
    emulator->registers[183] = 0xD2;
    emulator->register_pointer = 0;
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
    ESP_LOGI(TAG, "status registers 0 and 1 report ready/locked");
    ESP_LOGW(TAG, "current milestone supports one response byte per read request");
    return ESP_OK;
}
