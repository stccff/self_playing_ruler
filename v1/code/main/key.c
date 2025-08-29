#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "button_gpio.h"
#include "iot_button.h"
#include "gpio_pin_config.h"
#include "ruler.h"
#include "key.h"

#define TAG "key"
#define KEY_TASK_STACK_SIZE 4096


TaskHandle_t task_handle = NULL;

static void key_task(void *arg)
{
    int rc = ESP_OK;

    while (1) {
        uint32_t b_event;
        // 不清除进入时的位, 退出时清零, 通知值, 永久等待
        if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &b_event, portMAX_DELAY) == pdTRUE) {
            switch (b_event) {
            case BUTTON_SINGLE_CLICK:
                ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK");
                // do frequency measurement
                rc = freq_table_init(true); // force init
                if (rc != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to create frequency table");
                } else {
                    ESP_LOGI(TAG, "Frequency table initialized successfully");
                }
                break;
            default:
                ESP_LOGW(TAG, "Task: Unknown event %ld", b_event);
                break;
            }
            // resume button
            iot_button_resume();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void button_click_cb(void *arg, void *usr_data)
{
    // stop button
    iot_button_stop();

    // notify key task
    button_event_t b_event = *(button_event_t*)usr_data;
    int ret = xTaskNotify(task_handle, b_event, eSetValueWithoutOverwrite);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Notify key task failed");
    }
}

void button_init(void)
{
    // create gpio button
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = FREQ_TRIGG_IO,
        .active_level = 0,
    };
    button_handle_t gpio_btn = NULL;
    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn));

    xTaskCreate(key_task, "key_task", KEY_TASK_STACK_SIZE, NULL, 6, &task_handle);

    static button_event_t b_event = BUTTON_SINGLE_CLICK; // TODO: issue? user data without copy in iot_button_register_cb???
    ESP_ERROR_CHECK(iot_button_register_cb(gpio_btn, b_event, NULL, button_click_cb, &b_event));
}
