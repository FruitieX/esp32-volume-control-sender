/* ULP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include <esp_now.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "phy.h"
#include "esp_phy_init.h"
#include "esp_adc_cal.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */
static void init_ulp_program(void);

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program(void);

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static xQueueHandle s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 0;
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

// #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
// #endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Send send queue fail");
    }
}

uint16_t bat_mv;

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    buf->payload = ulp_last_result;

    // TinyPICO specific: read approx battery voltage
    // From https://github.com/UnexpectedMaker/tinypico-helper/blob/f6f992d3f36738796c21d6c7825b8ef641a5f31b/src/TinyPICO.cpp
    // Battery divider resistor values
    #define UPPER_DIVIDER 442
    #define LOWER_DIVIDER 160
    #define DEFAULT_VREF  1100  // Default reference voltage in mv
    #define BATT_CHANNEL ADC1_CHANNEL_7  // Battery voltage ADC input

    #define BAT_CHARGE 34
    #define BAT_VOLTAGE 35

    uint32_t raw;
    esp_adc_cal_characteristics_t chars;

    // grab latest voltage
    // analogRead(BAT_VOLTAGE);  // Just to get the ADC setup
    adc1_config_channel_atten(BATT_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    // this stops ULP readings from working
    raw = adc1_get_raw(BATT_CHANNEL);  // Read of raw ADC value

    // Get ADC calibration values
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_DEFAULT_VREF, &chars);

    // Convert to calibrated mV
    bat_mv = esp_adc_cal_raw_to_voltage(raw, &chars) * (LOWER_DIVIDER+UPPER_DIVIDER) / LOWER_DIVIDER;

    buf->bat_mv = bat_mv;

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_DO_SEND:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

                // ESP_LOGE(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                // ESP_LOGE(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);

                example_espnow_data_prepare(send_param);

                // printf("%d: Calling esp_now_send()\n", esp_log_early_timestamp());
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }

                break;
            }
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                // printf("%d: De-init wifi\n", esp_log_early_timestamp());
                // esp_wifi_stop();
                // esp_wifi_deinit();

                // Seems like we need to re-run this after using adc1_get_raw().
                adc1_ulp_enable();

                printf("%dms: Sent val=%d, bat_mv=%d. Entering deep sleep.\n", esp_log_early_timestamp(), ulp_last_result, bat_mv);
                start_ulp_program();
                ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
                esp_deep_sleep_start();

                /* Send the next data after the previous data is sent. */
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->len = 16;
    send_param->buffer = malloc(16);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}


static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

RTC_DATA_ATTR esp_phy_calibration_data_t rtc_cal_data;

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup\n");

        const esp_phy_init_data_t* init_data = esp_phy_get_init_data();
        if (init_data == NULL) {
            ESP_LOGE(TAG, "failed to obtain PHY init data");
            abort();
        }

        esp_phy_enable();

        // Full calibration on reset, store calibration data in RTC memory.
        register_chipv7_phy(init_data, &rtc_cal_data, PHY_RF_CAL_FULL);

        init_ulp_program();
        printf("Entering deep sleep\n\n");
        start_ulp_program();
        ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
        esp_deep_sleep_start();
    } else {
        ulp_last_result = ulp_result;
        // printf("%d: Deep sleep wakeup\n", esp_log_early_timestamp());
        // printf("ULP did %d measurements since last reset\n", ulp_sample_counter & UINT16_MAX);

        ulp_last_result &= UINT16_MAX;
        // printf("Value=%d\n", ulp_last_result);

        esp_phy_common_clock_enable();

        // No calibration on deep sleep wakeup, just set phy data from rtc_cal_data
        // printf("%d: esp_phy_get_init_data()\n", esp_log_early_timestamp());
        const esp_phy_init_data_t* init_data = esp_phy_get_init_data();
        if (init_data == NULL) {
            ESP_LOGE(TAG, "failed to obtain PHY init data");
            abort();
        }
        // printf("%d: register_chipv7_phy()\n", esp_log_early_timestamp());
        register_chipv7_phy(init_data, &rtc_cal_data, PHY_RF_CAL_NONE);

        phy_wakeup_init();

        // printf("%d: Initializing wifi\n", esp_log_early_timestamp());
        example_wifi_init();

        // printf("%d: Initializing espnow\n", esp_log_early_timestamp());
        example_espnow_init();

        example_espnow_event_t evt;
        example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
        evt.id = EXAMPLE_ESPNOW_DO_SEND;
        memcpy(send_cb->mac_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);

        // printf("%d: Adding msg to send queue\n", esp_log_early_timestamp());
        if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Send send queue fail");
        }
    }
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Configure ADC channel */
    /* Note: when changing channel here, also change 'adc_channel' constant
       in adc.S */
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
#if CONFIG_IDF_TARGET_ESP32
    adc1_config_width(ADC_WIDTH_BIT_12);
#elif CONFIG_IDF_TARGET_ESP32S2
    adc1_config_width(ADC_WIDTH_BIT_13);
#endif
    adc1_ulp_enable();

    /* Set ULP wake up period to 10ms */
    ulp_set_wakeup_period(0, 10 * 1000);

    // TinyPICO specific: disable onboard LED
    // From https://github.com/UnexpectedMaker/tinypico-helper/blob/f6f992d3f36738796c21d6c7825b8ef641a5f31b/src/TinyPICO.cpp
    #define DOTSTAR_PWR 13
    #define DOTSTAR_DATA 2
    #define DOTSTAR_CLK 12

    // digitalWrite( DOTSTAR_PWR, !state );
    // pinMode( DOTSTAR_DATA, state ? OUTPUT : INPUT_PULLDOWN );
    // pinMode( DOTSTAR_CLK, state ? OUTPUT : INPUT_PULLDOWN );

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << DOTSTAR_PWR);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(DOTSTAR_PWR, 1);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    io_conf.pin_bit_mask = ((1ULL << DOTSTAR_DATA) | (1ULL << DOTSTAR_CLK));
    gpio_config(&io_conf);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
#if CONFIG_IDF_TARGET_ESP32
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages
#endif
}

static void start_ulp_program(void)
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}
