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

/* ===================== CONFIGURATION ===================== */

#define TAG "FRAME_ANALYZER"

// Output mode selection:
// 0 = FRAME_DECODE_MODE (decode and print recognized frames)
// 1 = RAW_OUTPUT_MODE (print raw level changes with correct pairing)
#define OUTPUT_MODE 0

#define MONITOR_TX_GPIO     GPIO_NUM_13    // GPIO pin to monitor (remote TX / controller RX)
#define MONITOR_RX_GPIO  GPIO_NUM_12    // GPIO pin to monitor (controller TX / remote RX)
#define FRAME_GAP_US     6000           // Minimum gap between frames (µs)
#define BIT_US           104            // Microseconds per bit (calibrated from real remote)
#define BIT_TOLERANCE    35             // Tolerance for bit timing (±35%)
#define FRAME_BITS       48             // Expected bits per frame
#define MIN_PULSE_US     50             // Minimum pulse width (noise filter)

#define FRAME_QUEUE_SIZE 10             // Max frames in queue
#define POLLING_CORE     0              // Core for GPIO polling (time-critical)
#define DECODE_CORE      1              // Core for frame decoding (non-critical)

/* ===================== DATA STRUCTURES ===================== */

typedef struct {
    uint8_t bytes[6];
    size_t bit_count;
    uint8_t channel;  // 0 = TX (remote->controller), 1 = RX (controller->remote)
} frame_data_t;

typedef struct {
    const char *name;
    uint8_t bytes[6];
} frame_def_t;

// Global queue handle
static QueueHandle_t frame_queue = NULL;

// Database of recognized remote control button frames
static const frame_def_t KNOWN_FRAMES[] = {
    { "IDLE",     { 0x4A, 0x03, 0x08, 0xA0, 0x80, 0x05 } },  // Idle state (last byte varies)
    { "PRESET_1", { 0x4A, 0x03, 0x48, 0xA0, 0x80, 0x06 } },
    { "PRESET_2", { 0x4A, 0x03, 0x88, 0xA0, 0x80, 0x0A } },
    { "UP",       { 0x4A, 0x03, 0x08, 0xA4, 0x80, 0x42 } },
    { "DOWN",     { 0x4A, 0x03, 0x08, 0xA8, 0x80, 0x82 } },
};

#define NUM_KNOWN_FRAMES (sizeof(KNOWN_FRAMES) / sizeof(KNOWN_FRAMES[0]))

/* ===================== HELPER FUNCTIONS ===================== */

/**
 * Find frame name in known frames database
 * Returns frame name if found, NULL otherwise
 * Note: IDLE frame ignores last byte, others match all 6 bytes
 */
static const char* identify_frame(const uint8_t *frame)
{
    for (size_t i = 0; i < NUM_KNOWN_FRAMES; i++) {
        // IDLE frame: compare only first 5 bytes (last byte varies)
        if (strcmp(KNOWN_FRAMES[i].name, "IDLE") == 0) {
            if (memcmp(frame, KNOWN_FRAMES[i].bytes, 5) == 0) {
                return KNOWN_FRAMES[i].name;
            }
        } else {
            // All other frames: compare all 6 bytes
            if (memcmp(frame, KNOWN_FRAMES[i].bytes, 6) == 0) {
                return KNOWN_FRAMES[i].name;
            }
        }
    }
    return NULL;
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
    gpio_reset_pin(MONITOR_TX_GPIO);
    gpio_reset_pin(MONITOR_RX_GPIO);

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << MONITOR_TX_GPIO) | (1ULL << MONITOR_RX_GPIO),
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
    frame_data_t frame_data;
    uint8_t last_frame_tx[6] = {0};
    uint32_t repeat_count_tx = 0;
    bool has_last_tx = false;
    
    uint8_t last_frame_rx[6] = {0};
    uint32_t repeat_count_rx = 0;
    bool has_last_rx = false;
    
    ESP_LOGI(TAG, "Decoder task running on core %d", xPortGetCoreID());
    
    while (1) {
        if (xQueueReceive(frame_queue, &frame_data, portMAX_DELAY)) {
            const char *channel_str = (frame_data.channel == 0) ? "TX" : "RX";
            
            if (frame_data.bit_count != FRAME_BITS) {
                // Finish current line before warning
                if ((frame_data.channel == 0 && has_last_tx && repeat_count_tx > 0) ||
                    (frame_data.channel == 1 && has_last_rx && repeat_count_rx > 0)) {
                    printf("\n");
                }
                
                ESP_LOGW(TAG, "Incomplete frame (%s): %zu bits (expected %d)", 
                         channel_str, frame_data.bit_count, FRAME_BITS);
                
                if (frame_data.channel == 0) {
                    has_last_tx = false;
                    repeat_count_tx = 0;
                } else {
                    has_last_rx = false;
                    repeat_count_rx = 0;
                }
            } else {
                if (frame_data.channel == 0) {
                    // TX channel
                    if (has_last_tx && memcmp(frame_data.bytes, last_frame_tx, 5) == 0) {
                        repeat_count_tx++;
                        const char *frame_name = identify_frame(last_frame_tx);
                        if (frame_name) {
                            printf("\r[TX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu) [%s]     ",
                                   last_frame_tx[0], last_frame_tx[1], last_frame_tx[2],
                                   last_frame_tx[3], last_frame_tx[4], last_frame_tx[5],
                                   (unsigned long)repeat_count_tx, frame_name);
                        } else {
                            printf("\r[TX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu)     ",
                                   last_frame_tx[0], last_frame_tx[1], last_frame_tx[2],
                                   last_frame_tx[3], last_frame_tx[4], last_frame_tx[5],
                                   (unsigned long)repeat_count_tx);
                        }
                    } else {
                        if (has_last_tx) {
                            printf("\n");
                        }
                        memcpy(last_frame_tx, frame_data.bytes, 6);
                        repeat_count_tx = 1;
                        has_last_tx = true;
                        
                        const char *frame_name = identify_frame(last_frame_tx);
                        if (frame_name) {
                            printf("[TX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu) [%s]",
                                   last_frame_tx[0], last_frame_tx[1], last_frame_tx[2],
                                   last_frame_tx[3], last_frame_tx[4], last_frame_tx[5],
                                   (unsigned long)repeat_count_tx, frame_name);
                        } else {
                            printf("[TX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu)",
                                   last_frame_tx[0], last_frame_tx[1], last_frame_tx[2],
                                   last_frame_tx[3], last_frame_tx[4], last_frame_tx[5],
                                   (unsigned long)repeat_count_tx);
                        }
                    }
                } else {
                    // RX channel
                    if (has_last_rx && memcmp(frame_data.bytes, last_frame_rx, 6) == 0) {
                        repeat_count_rx++;
                        const char *frame_name = identify_frame(last_frame_rx);
                        if (frame_name) {
                            printf("\r[RX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu) [%s]     ",
                                   last_frame_rx[0], last_frame_rx[1], last_frame_rx[2],
                                   last_frame_rx[3], last_frame_rx[4], last_frame_rx[5],
                                   (unsigned long)repeat_count_rx, frame_name);
                        } else {
                            printf("\r[RX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu)     ",
                                   last_frame_rx[0], last_frame_rx[1], last_frame_rx[2],
                                   last_frame_rx[3], last_frame_rx[4], last_frame_rx[5],
                                   (unsigned long)repeat_count_rx);
                        }
                    } else {
                        if (has_last_rx) {
                            printf("\n");
                        }
                        memcpy(last_frame_rx, frame_data.bytes, 6);
                        repeat_count_rx = 1;
                        has_last_rx = true;
                        
                        const char *frame_name = identify_frame(last_frame_rx);
                        if (frame_name) {
                            printf("[RX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu) [%s]",
                                   last_frame_rx[0], last_frame_rx[1], last_frame_rx[2],
                                   last_frame_rx[3], last_frame_rx[4], last_frame_rx[5],
                                   (unsigned long)repeat_count_rx, frame_name);
                        } else {
                            printf("[RX] FRAME: %02X %02X %02X %02X %02X %02X (x%lu)",
                                   last_frame_rx[0], last_frame_rx[1], last_frame_rx[2],
                                   last_frame_rx[3], last_frame_rx[4], last_frame_rx[5],
                                   (unsigned long)repeat_count_rx);
                        }
                    }
                }
            }
        }
    }
}

/**
 * GPIO polling task - runs on dedicated core
 * Captures pulses on both TX and RX lines simultaneously
 */
static void polling_task(void *arg)
{
    ESP_LOGI(TAG, "Polling task running on core %d", xPortGetCoreID());
    
    init_gpio();

    // TX channel state
    uint32_t last_level_tx = gpio_read_fast(MONITOR_TX_GPIO);
    int64_t last_time_tx = esp_timer_get_time();
    uint8_t current_frame_tx[6] = {0};
    size_t bit_counter_tx = 0;
    bool started_tx = false;

    // RX channel state
    uint32_t last_level_rx = gpio_read_fast(MONITOR_RX_GPIO);
    int64_t last_time_rx = esp_timer_get_time();
    uint8_t current_frame_rx[6] = {0};
    size_t bit_counter_rx = 0;
    bool started_rx = false;

    // Half-duplex verification: track when channels are actively transmitting
    bool tx_is_active = false;  // TX channel transmitting a frame
    bool rx_is_active = false;  // RX channel transmitting a frame
    size_t collision_count = 0; // Count simultaneous transmissions
    int64_t last_status_time = esp_timer_get_time();
    size_t tx_frame_count = 0;
    size_t rx_frame_count = 0;

    // Main polling loop
    while (1) {
        uint32_t level_tx = gpio_read_fast(MONITOR_TX_GPIO);
        uint32_t level_rx = gpio_read_fast(MONITOR_RX_GPIO);
        int64_t now = esp_timer_get_time();

        // Periodic status report (every 30 seconds)
        if ((now - last_status_time) >= 30000000) {
            if (collision_count == 0) {
                ESP_LOGI(TAG, "✓ Half-duplex verified: TX frames=%zu, RX frames=%zu, Collisions=0", 
                         tx_frame_count, rx_frame_count);
            } else {
                ESP_LOGW(TAG, "Half-duplex status: TX frames=%zu, RX frames=%zu, Collisions=%zu", 
                         tx_frame_count, rx_frame_count, collision_count);
            }
            last_status_time = now;
        }

        // Process TX channel transitions
        if (level_tx != last_level_tx) {
            uint32_t pulse_level = last_level_tx;
            uint32_t pulse_duration = (uint32_t)(now - last_time_tx);
            last_time_tx = now;

            if (pulse_duration < MIN_PULSE_US) {
                last_level_tx = level_tx;
            } else if (!started_tx && pulse_duration < FRAME_GAP_US) {
                last_level_tx = level_tx;
            } else {
                started_tx = true;

                if (pulse_duration >= FRAME_GAP_US) {
                    if (bit_counter_tx > 0) {
                        frame_data_t frame_data;
                        memcpy(frame_data.bytes, current_frame_tx, 6);
                        frame_data.bit_count = bit_counter_tx;
                        frame_data.channel = 0;  // TX
                        xQueueSend(frame_queue, &frame_data, 0);
                        tx_frame_count++;
                    }
                    memset(current_frame_tx, 0, 6);
                    bit_counter_tx = 0;
                    tx_is_active = false;  // TX frame complete
                    last_level_tx = level_tx;
                } else {
                    size_t nb_bits = (pulse_duration * 100 + (BIT_US * BIT_TOLERANCE)) / (BIT_US * 100);
                    for (size_t i = 0; i < nb_bits; i++) {
                        if (bit_counter_tx >= FRAME_BITS) break;
                        size_t byte_index = bit_counter_tx >> 3;
                        size_t bit_index = bit_counter_tx & 7;
                        if (pulse_level) {
                            current_frame_tx[byte_index] |= (1 << bit_index);
                        }
                        bit_counter_tx++;
                    }
                    tx_is_active = true;  // TX is transmitting
                    
                    // Check for half-duplex violation
                    if (rx_is_active && level_rx == 0) {
                        collision_count++;
                        ESP_LOGW(TAG, "⚠️  Half-duplex violation detected! Both TX and RX active simultaneously (collision #%zu)", collision_count);
                    }
                    
                    last_level_tx = level_tx;
                }
            }
        }

        // Process RX channel transitions
        if (level_rx != last_level_rx) {
            uint32_t pulse_level = last_level_rx;
            uint32_t pulse_duration = (uint32_t)(now - last_time_rx);
            last_time_rx = now;

            if (pulse_duration < MIN_PULSE_US) {
                last_level_rx = level_rx;
            } else if (!started_rx && pulse_duration < FRAME_GAP_US) {
                last_level_rx = level_rx;
            } else {
                started_rx = true;

                if (pulse_duration >= FRAME_GAP_US) {
                    if (bit_counter_rx > 0) {
                        frame_data_t frame_data;
                        memcpy(frame_data.bytes, current_frame_rx, 6);
                        frame_data.bit_count = bit_counter_rx;
                        frame_data.channel = 1;  // RX
                        xQueueSend(frame_queue, &frame_data, 0);
                        rx_frame_count++;
                    }
                    memset(current_frame_rx, 0, 6);
                    bit_counter_rx = 0;
                    rx_is_active = false;  // RX frame complete
                    last_level_rx = level_rx;
                } else {
                    size_t nb_bits = (pulse_duration * 100 + (BIT_US * BIT_TOLERANCE)) / (BIT_US * 100);
                    for (size_t i = 0; i < nb_bits; i++) {
                        if (bit_counter_rx >= FRAME_BITS) break;
                        size_t byte_index = bit_counter_rx >> 3;
                        size_t bit_index = bit_counter_rx & 7;
                        if (pulse_level) {
                            current_frame_rx[byte_index] |= (1 << bit_index);
                        }
                        bit_counter_rx++;
                    }
                    rx_is_active = true;  // RX is transmitting
                    
                    // Check for half-duplex violation
                    if (tx_is_active && level_tx == 0) {
                        collision_count++;
                        ESP_LOGW(TAG, "⚠️  Half-duplex violation detected! Both TX and RX active simultaneously (collision #%zu)", collision_count);
                    }
                    
                    last_level_rx = level_rx;
                }
            }
        }
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
    xTaskCreatePinnedToCore(
        decoder_task,
        "decoder",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,  // High priority (separate core)
        NULL,
        DECODE_CORE
    );
    #endif
    
    // Create polling task on POLLING_CORE  
    xTaskCreatePinnedToCore(
        polling_task,
        "polling",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,  // Highest priority
        NULL,
        POLLING_CORE
    );
}
