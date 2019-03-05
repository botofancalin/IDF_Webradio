
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "http.h"
#include "driver/i2s.h"

#include "vector.h"
#include "esp32_fifo.h"
#include "audio_renderer.h"
#include "web_radio.h"
#include "wifi.h"
#include "driver/gpio.h"

#define WIFI_LIST_NUM 10

#define TAG "main"

//Priorities of the reader and the decoder thread. bigger number = higher prio
#define PRIO_READER configMAX_PRIORITIES - 3
#define PRIO_MQTT configMAX_PRIORITIES - 3
#define PRIO_CONNECT configMAX_PRIORITIES - 1

static void init_hardware()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //Initialize the SPI RAM chip communications and see if it actually retains some bytes. If it
    //doesn't, warn user.
    if (!FifoInit())
    {
        printf("\n\nSPI RAM chip fail!\n");
    }

    ESP_LOGI(TAG, "hardware initialized");
}

static void start_wifi()
{
    ESP_LOGI(TAG, "starting network");

    /* FreeRTOS event group to signal when we are connected & ready to make a request */
    EventGroupHandle_t wifi_event_group = xEventGroupCreate();

    /* init wifi */
    initialise_wifi(wifi_event_group);

    /* Wait for the callback to set the CONNECTED_BIT in the event group. */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);
}

static void start_web_radio()
{
    // init web radio
    web_radio_t *radio_config = calloc(1, sizeof(web_radio_t));
    radio_config->playlist = playlist_create();
    playlist_load_pls(radio_config->playlist);

    // init player config
    radio_config->player_config = calloc(1, sizeof(player_t));
    radio_config->player_config->command = CMD_NONE;
    radio_config->player_config->decoder_status = UNINITIALIZED;
    radio_config->player_config->decoder_command = CMD_NONE;
    radio_config->player_config->buffer_pref = BUF_PREF_FAST;
    radio_config->player_config->media_stream = calloc(1, sizeof(media_stream_t));

    // init renderer
    renderer_init();

    // start radio
    web_radio_init(radio_config);
    web_radio_start(radio_config);
}

/**
 * entry point
 */
void app_main()
{
    ESP_LOGI(TAG, "starting app_main()");
    ESP_LOGI(TAG, "RAM left: %u", esp_get_free_heap_size());

    init_hardware();
    gpio_pad_select_gpio(GPIO_NUM_27);
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_27, 0);

#ifdef CONFIG_BT_SPEAKER_MODE
    bt_speaker_start(create_renderer_config());
#else
    start_wifi();
    start_web_radio();
#endif

    ESP_LOGI(TAG, "RAM left %d", esp_get_free_heap_size());
    //ESP_LOGI(TAG, "app_main stack: %d\n", uxTaskGetStackHighWaterMark(NULL));
}
