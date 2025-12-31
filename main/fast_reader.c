/*
 * GPIO Frame Analyzer for TV Remote Protocol
 * 
 * Decodes pulse-width encoded frames from a TV remote control signal.
 * Uses polling-based GPIO sampling at 240 MHz CPU frequency.
 * 
 * Protocol:
 * - 48-bit frames (6 bytes)
 * - Pulse width encoding: ~102µs per bit
 * - Frame gap: ≥6000µs between frames
 * - Bit encoding: pulse duration determines number of consecutive bits
 * 
 * Features:
 * - Dual-channel monitoring: TX (remote→controller) and RX (controller→remote)
 * - Half-duplex verification: detects simultaneous transmissions (protocol violations)
 * - Frame identification: matches captured frames against known button patterns
 * - Periodic status reports: shows frame counts and collision detection
 */

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

/* ===================== CONFIGURATION ===================== */

#define TAG "FRAME_ANALYZER"

// Output mode selection:
// 0 = FRAME_DECODE_MODE (decode and print recognized frames)
// 1 = RAW_OUTPUT_MODE (print raw level changes with correct pairing)
#define OUTPUT_MODE 1

#define TASK_PRIORITY   (configMAX_PRIORITIES - 1)  // High priority for time-critical tasks
#define MONITOR_TX_GPIO_OUT     GPIO_NUM_13    // GPIO pin to monitor (remote TX / controller RX)
#define MONITOR_RX_GPIO_IN  GPIO_NUM_12    // GPIO pin to monitor (controller TX / remote RX)

/* ===================== GPIO FUNCTIONS ===================== */

/**
 * Fast GPIO read using direct register access
 * Faster than gpio_get_level() for tight polling loops
 */
static inline uint32_t gpio_read_fast(uint8_t gpio_num)
{
    return (GPIO.in >> gpio_num) & 0x1;
}

/**
 * Initialize GPIO pin for input monitoring
 * Configures as input with no pull resistors
 */
static void init_gpio(void)
{
    gpio_reset_pin(MONITOR_TX_GPIO_OUT);
    gpio_reset_pin(MONITOR_RX_GPIO_IN);

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << MONITOR_TX_GPIO_OUT) | (1ULL << MONITOR_RX_GPIO_IN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&cfg));
}

// Standalone event struct and queue
#define FRAME_QUEUE_SIZE 512
typedef struct {
    uint32_t tx_level;
    int64_t tx_time;
    uint32_t rx_level;
    int64_t rx_time;
} tx_event_t;
static QueueHandle_t frame_queue = NULL;

void poll_task(void *arg)
{
    uint32_t last_level_tx = gpio_read_fast(MONITOR_TX_GPIO_OUT);
    uint32_t last_level_rx = gpio_read_fast(MONITOR_RX_GPIO_IN);
    int64_t last_time = esp_timer_get_time();
    bool add_to_queue = false;
    while (1) {
        uint32_t level_tx = gpio_read_fast(MONITOR_TX_GPIO_OUT);
        uint32_t level_rx = gpio_read_fast(MONITOR_RX_GPIO_IN);
        int64_t tx_now = esp_timer_get_time();
        int64_t rx_now = tx_now;

        if (level_tx != last_level_tx) {
            tx_event_t event = {
                .tx_level = last_level_tx,
                .tx_time = tx_now - last_time
            };

            last_time = now;
            last_level_tx = level_tx;
            add_to_queue = true;
        }

        if (level_rx != last_level_rx) {
            tx_event_t event = {
                .tx_level = last_level_tx,
                .tx_time = tx_now - last_time
            };

            last_time = now;
            last_level_tx = level_tx;
            add_to_queue = true;
        }

        if (add_to_queue == false) {
            continue;
        }
        if (xQueueSend(frame_queue, &event, 0) != pdTRUE) {
            printf("[ERROR] Queue full, event lost!\n");
        }
    }
}

void print_task(void *arg)
{
    tx_event_t event;
    char buf[48];
    while (1) {
        if (xQueueReceive(frame_queue, &event, portMAX_DELAY)) {
            int len = snprintf(buf, sizeof(buf), "%lu %lld\n", event.tx_level, event.duration_us);
            if (len > 0) {
                // Write to stdout (fd 1)
                write(STDOUT_FILENO, buf, len);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting TV Remote Frame Analyzer");
    
    // Disable stdout buffering for real-time updates
    setvbuf(stdout, NULL, _IONBF, 0);
    

    frame_queue = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(tx_event_t));
    if (!frame_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return;
    }

    ESP_LOGI(TAG, "Polling task running on core %d", xPortGetCoreID());
    
    init_gpio();

    // Start logging task
    xTaskCreatePinnedToCore(print_task, "print_task", 4096, NULL, TASK_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(poll_task, "poll_task", 4096, NULL, TASK_PRIORITY, NULL, 1);
}
