#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "controls.h"

static xQueueHandle gpio_evt_queue = NULL;
static TaskHandle_t *gpio_task;
#define ESP_INTR_FLAG_DEFAULT 0

/* gpio event handler */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t gpio_num = (uint32_t)arg;

    xQueueSendToBackFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void controls_init(TaskFunction_t gpio_handler_task, const uint16_t usStackDepth, void *user_data)
{
    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pin, use GPIO here
    io_conf.pin_bit_mask = (1 << CONTROL_PIN);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    gpio_handler_param_t *params = calloc(1, sizeof(gpio_handler_param_t));
    params->gpio_evt_queue = gpio_evt_queue;
    params->user_data = user_data;

    //start gpio task
    xTaskCreatePinnedToCore(gpio_handler_task, "gpio_handler_task", usStackDepth, params, 10, gpio_task, 0);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // remove existing handler that may be present
    gpio_isr_handler_remove(CONTROL_PIN);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONTROL_PIN, gpio_isr_handler, (void *)CONTROL_PIN);
}

void controls_destroy()
{
    gpio_isr_handler_remove(CONTROL_PIN);
    vTaskDelete(gpio_task);
    vQueueDelete(gpio_evt_queue);
    // TODO: free gpio_handler_param_t params
}
