#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "uart_ctrl.h"
#include "terminal.h"

/* UART hardware configuration */
#define UART_PORT_NUM    (0)
#define UART_BAUD_RATE   (115200)
#define UART_TXD         (43)
#define UART_RXD         (44)
#define UART_RTS         (UART_PIN_NO_CHANGE)
#define UART_CTS         (UART_PIN_NO_CHANGE)

#define TASK_STACK_SIZE  (4096)
#define READ_BUF_SIZE    (64)

static const char *TAG = "UART_CTRL";

static void uart_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 1024 * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));

    ESP_LOGI(TAG, "UART terminal ready, baud=%d", UART_BAUD_RATE);

    terminal_prompt();

    char buf[READ_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, READ_BUF_SIZE, pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++) {
            terminal_feed(buf[i]);
        }
    }
}

void uart_init(void)
{
    xTaskCreate(uart_task, "uart_task", TASK_STACK_SIZE, NULL, 5, NULL);
}
