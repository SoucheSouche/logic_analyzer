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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "rx_frames_heights.h"

/* ===================== CONFIGURATION ===================== */

#define TAG "FRAME_ANALYZER"

// Output mode selection:
// 0 = FRAME_DECODE_MODE (decode and print recognized frames)
// 1 = RAW_OUTPUT_MODE (print raw level changes with correct pairing)
#define OUTPUT_MODE 0

#define TASK_PRIORITY   (configMAX_PRIORITIES - 1)
#define MONITOR_TX_GPIO_OUT  GPIO_NUM_13
#define MONITOR_RX_GPIO_IN  GPIO_NUM_12
#define FRAME_GAP_US_TX  7000
#define FRAME_GAP_US_RX  5000
#define BIT_US           103
#define BIT_TOLERANCE    1
#define FRAME_BITS_TX    48
#define FRAME_BITS_RX    56
#define FRAME_SIZE_TX    6
#define FRAME_SIZE_RX    7
#define FRAME_QUEUE_SIZE 128
#define POLLING_CORE     1
#define DECODE_CORE      0

/* ===================== DATA STRUCTURES ===================== */
typedef struct {
    uint8_t bytes[7];
    size_t bit_count;
    uint8_t channel;  // 0 = TX (remote->controller), 1 = RX (controller->remote)
} frame_data_t;

// Global queue handle
static QueueHandle_t frame_queue = NULL;

/* ===================== HELPER FUNCTIONS ===================== */

// Identify RX frame height by matching bytes 1-4 of the received frame
static float identify_rx_frame_height(const uint8_t *frame)
{
    // for (size_t i = 0; i < RX_FRAMES_HEIGHTS_COUNT; i++) {
    //     if (memcmp(&frame[1], rx_frames_heights[i].frame, 4) == 0) {
    //         return rx_frames_heights[i].height;
    //     }
    // }
    // return -1.0f;
    return -1;
}

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

/* ===================== MAIN APPLICATION ===================== */

/**
 * Decoder task - runs on separate core
 * Receives frames from queue and decodes/displays them
 */
static void decoder_task(void *arg)
{
    ESP_LOGI(TAG, "Decoder task running on core %d", xPortGetCoreID());
    
    while (1) {
        frame_data_t frame_data;
        if (xQueueReceive(frame_queue, &frame_data, portMAX_DELAY)) {
            if (frame_data.channel == 0) {
                /* tx */
                printf("[TX] %02x %02x %02x %02x %02x %02x\n",
                       frame_data.bytes[0], frame_data.bytes[1], frame_data.bytes[2],
                       frame_data.bytes[3], frame_data.bytes[4], frame_data.bytes[5]);
            }
            // else {
            //     /* rx */
            //     printf("[RX] %02x %02x %02x %02x %02x %02x %02x\n",
            //            frame_data.bytes[0], frame_data.bytes[1], frame_data.bytes[2],
            //            frame_data.bytes[3], frame_data.bytes[4], frame_data.bytes[5],
            //            frame_data.bytes[6]);
            // }
        }
    }
}

static void polling_task(void *arg)
{
    ESP_LOGI(TAG, "Polling task running on core %d", xPortGetCoreID());
    init_gpio();

    // TX channel state
    uint32_t last_level_tx = gpio_read_fast(MONITOR_TX_GPIO_OUT);
    int64_t last_time_tx = esp_timer_get_time();
    uint8_t current_frame_tx[6] = {0};
    size_t bit_counter_tx = 0;
    bool started_tx = false;
    size_t skip_bits_tx = 1;

    // RX channel state
    uint32_t last_level_rx = gpio_read_fast(MONITOR_RX_GPIO_IN);
    int64_t last_time_rx = esp_timer_get_time();
    uint8_t current_frame_rx[6] = {0};
    size_t bit_counter_rx = 0;
    bool started_rx = false;
    size_t skip_bits_rx = 2;

    // Half-duplex verification: track when channels are actively transmitting
    bool tx_is_active = false;  // TX channel transmitting a frame
    bool rx_is_active = false;  // RX channel transmitting a frame
    size_t tx_frame_count = 0;
    size_t rx_frame_count = 0;

    // Main polling loop
    while (1) {
#if OUTPUT_MODE == 1
        printf("%ld, %ld, %lld\n", gpio_read_fast(MONITOR_TX_GPIO_OUT), gpio_read_fast(MONITOR_RX_GPIO_IN), esp_timer_get_time());
#else
        uint32_t level_tx = gpio_read_fast(MONITOR_TX_GPIO_OUT);
        uint32_t level_rx = gpio_read_fast(MONITOR_RX_GPIO_IN);
        int64_t now = esp_timer_get_time();

        if (level_tx != last_level_tx) {
            // Process TX channel transition
            uint32_t pulse_level = last_level_tx;
            uint32_t pulse_duration = (uint32_t)(now - last_time_tx);
            last_time_tx = now;

            if (!started_tx && pulse_duration < FRAME_GAP_US_TX) {
                last_level_tx = level_tx;
            } else {
                started_tx = true;

                if (pulse_duration >= FRAME_GAP_US_TX) {
                    if (bit_counter_tx == FRAME_BITS_TX) {
                        frame_data_t frame_data;
                        memcpy(frame_data.bytes, current_frame_tx, 6);
                        frame_data.bit_count = bit_counter_tx;
                        frame_data.channel = 0; // TX
                        if (xQueueSend(frame_queue, &frame_data, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "Frame queue full! Frame lost.");
                        }
                        tx_frame_count++;
                    } else {
                        ESP_LOGW(TAG, "Incomplete TX frame: received %zu bits, expected %d", 
                             bit_counter_tx, FRAME_BITS_TX);
                    }
                    memset(current_frame_tx, 0, 6);
                    bit_counter_tx = 0;
                    tx_is_active = false;
                    last_level_tx = level_tx;
                    skip_bits_tx = 1;
                } else {
                    size_t nb_bits = (pulse_duration * 100 + (BIT_US * BIT_TOLERANCE)) / (BIT_US * 100);
                    int i = 0;
                    if (skip_bits_tx > 0) {
                        size_t to_skip = (nb_bits < (size_t)skip_bits_tx) ? nb_bits : (size_t)skip_bits_tx;
                        // Explicitly do nothing for these bits (they remain 0)
                        i += to_skip;
                        skip_bits_tx -= to_skip;
                        nb_bits -= to_skip;
                    }
                    for (; i < nb_bits; i++) {
                        if (bit_counter_tx >= FRAME_BITS_TX) {
                            ESP_LOGW(TAG, "Frame overrun on TX: received %zu bits, expected max %d", 
                                 bit_counter_tx + 1, FRAME_BITS_TX);
                            break;
                        }
                        size_t byte_index = bit_counter_tx >> 3;
                        size_t bit_index = bit_counter_tx & 7;
                        if (pulse_level) {
                            current_frame_tx[byte_index] |= (1 << bit_index);
                        }
                        bit_counter_tx++;
                    }
                    tx_is_active = true;
                    // Check for half-duplex violation
                    if (rx_is_active && level_rx == 0) {
                        ESP_LOGW(TAG, "⚠️  Half-duplex violation detected! Both TX and RX active simultaneously");
                    }
                    last_level_tx = level_tx;
                }
            }
        }

        // if (level_rx != last_level_rx) {
        //     // Process RX channel transition
        //     uint32_t pulse_level = last_level_rx;
        //     uint32_t pulse_duration = (uint32_t)(now - last_time_rx);
        //     last_time_rx = now;

        //     if (!started_rx && pulse_duration < FRAME_GAP_US_RX) {
        //         last_level_rx = level_rx;
        //     } else {
        //         started_rx = true;

        //         if (pulse_duration >= FRAME_GAP_US_RX) {
        //             if (bit_counter_rx == FRAME_BITS_RX) {
        //                 frame_data_t frame_data;
        //                 memcpy(frame_data.bytes, current_frame_rx, 7);
        //                 frame_data.bit_count = bit_counter_rx;
        //                 frame_data.channel = 1; // RX
        //                 if (xQueueSend(frame_queue, &frame_data, 0) != pdTRUE) {
        //                     ESP_LOGW(TAG, "Frame queue full! Frame lost.");
        //                 }
        //                 rx_frame_count++;
        //             } else {
        //                 ESP_LOGW(TAG, "Incomplete RX frame: received %zu bits, expected %d", 
        //                      bit_counter_rx, FRAME_BITS_RX);
        //             }
        //             memset(current_frame_rx, 0, 7);
        //             bit_counter_rx = 0;
        //             rx_is_active = false;
        //             last_level_rx = level_rx;
        //             skip_bits_rx = 2;
        //         } else {
        //             size_t nb_bits = (pulse_duration * 100 + (BIT_US * BIT_TOLERANCE)) / (BIT_US * 100);
        //             int i = 0;
        //             if (skip_bits_rx > 0) {
        //                 size_t to_skip = (nb_bits < (size_t)skip_bits_rx) ? nb_bits : (size_t)skip_bits_rx;
        //                 // Explicitly do nothing for these bits (they remain 0)
        //                 i += to_skip;
        //                 skip_bits_rx -= to_skip;
        //                 nb_bits -= to_skip;
        //             }
        //             for (; i < nb_bits; i++) {
        //                 if (bit_counter_rx >= FRAME_BITS_RX) {
        //                     ESP_LOGW(TAG, "Frame overrun on RX: received %zu bits, expected max %d", 
        //                          bit_counter_rx + 1, FRAME_BITS_RX);
        //                     break;
        //                 }
        //                 size_t byte_index = bit_counter_rx >> 3;
        //                 size_t bit_index = bit_counter_rx & 7;
        //                 if (pulse_level) {
        //                     current_frame_rx[byte_index] |= (1 << bit_index);
        //                 }
        //                 bit_counter_rx++;
        //             }
        //             rx_is_active = true;
        //             // Check for half-duplex violation
        //             if (tx_is_active && level_tx == 0) {
        //                 ESP_LOGW(TAG, "⚠️  Half-duplex violation detected! Both TX and RX active simultaneously");
        //             }
        //             last_level_rx = level_rx;
        //         }
        //     }
        // }
#endif
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting TV Remote Frame Analyzer");
    
    // Disable stdout buffering for real-time updates
    setvbuf(stdout, NULL, _IONBF, 0);
    
    #if OUTPUT_MODE == 0
    // Create queue for frame passing
    frame_queue = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(frame_data_t));
    if (!frame_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return;
    }
    
    // Create decoder task on DECODE_CORE
    xTaskCreatePinnedToCore( decoder_task, "decoder", 4096, NULL, TASK_PRIORITY, NULL, DECODE_CORE);
    #endif

    xTaskCreatePinnedToCore( polling_task, "polling", 4096, NULL, TASK_PRIORITY, NULL, POLLING_CORE);
}
