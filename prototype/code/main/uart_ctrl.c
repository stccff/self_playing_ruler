#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "uart_ctrl.h"
#include "math.h"

// uart configurations
#define ECHO_TEST_TXD (43)
#define ECHO_TEST_RXD (44)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (0)
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (1024 * 10)


static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)


double k = 229049.0652;
#define TOTAL_LENGTH 44.5   // mm
#define TOTAL_POS 300       // steps
#define LENGHT_MIN 13.5     // mm
// #define LENGHT_MAX 57.5     // mm



static int convert_note_to_pos(char *note);
static int parse_jianpu(int base, const char *note);


static void uart_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
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

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    int total_len = 0;
    int lastpos = 150;
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, &data[total_len], (BUF_SIZE - total_len - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            total_len += len;
            if (data[total_len - 1] != '\n' && data[total_len - 1] != '\r') {
                continue;
            } else {
                data[total_len] = '\0';
                total_len = 0;
            }
        } else {
            continue;
        }
        ESP_LOGI(TAG, "data = %s", data);
        
        // int note = 0;
        // sscanf((char *)data, "%d", &note);
        int pos = convert_note_to_pos((char *)data);

        // send signal to fret motor
        xTaskNotify(get_servo_motor_task_handle(), 1, eSetValueWithOverwrite);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // send signal to step motor
        xTaskNotify(get_step_motor_task_handle(), (uint32_t)pos, eSetValueWithOverwrite);
        
        vTaskDelay(300 * abs(pos - lastpos) / TOTAL_POS / portTICK_PERIOD_MS);
        lastpos = pos;
        // send signal to fret motor
        xTaskNotify(get_servo_motor_task_handle(), 2, eSetValueWithOverwrite);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // send signal to strum motor        
        xTaskNotify(get_servo_motor_task_handle(), 3, eSetValueWithOverwrite);
    }   
}

void uart_init(void)
{
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 3, NULL);
}

// *****************************************************************************************************************
double g_midi_freq[128] = {
    8.18,
    8.66,
    9.18,
    9.72,
    10.3,
    10.91,
    11.56,
    12.25,
    12.98,
    13.75,
    14.57,
    15.43,
    16.35,
    17.32,
    18.35,
    19.45,
    20.6,
    21.83,
    23.12,
    24.5,
    25.96,
    27.5,
    29.14,
    30.87,
    32.7,
    34.65,
    36.71,
    38.89,
    41.2,
    43.65,
    46.25,
    49,
    51.91,
    55,
    58.27,
    61.74,
    65.41,
    69.3,
    73.42,
    77.78,
    82.41,
    87.31,
    92.5,
    98,
    103.83,
    110,
    116.54,
    123.47,
    130.81,
    138.59,
    146.83,
    155.56,
    164.81,
    174.61,
    185,
    196,
    207.65,
    220,
    233.08,
    246.94,
    261.63,
    277.18,
    293.66,
    311.13,
    329.63,
    349.23,
    369.99,
    392,
    415.3,
    440,
    466.16,
    493.88,
    523.25,
    554.37,
    587.33,
    622.25,
    659.26,
    698.46,
    739.99,
    783.99,
    830.61,
    880,
    932.33,
    987.77,
    1046.5,
    1108.73,
    1174.66,
    1244.51,
    1318.51,
    1396.91,
    1479.98,
    1567.98,
    1661.22,
    1760,
    1864.66,
    1975.53,
    2093,
    2217.46,
    2349.32,
    2489.02,
    2637.02,
    2793.83,
    2959.96,
    3135.96,
    3322.44,
    3520,
    3729.31,
    3951.07,
    4186.01,
    4434.92,
    4698.64,
    4978.03,
    5274.04,
    5587.65,
    5919.91,
    6271.93,
    6644.88,
    7040,
    7458.62,
    7902.13,
    8372.02,
    8869.84,
    9397.27,
    9956.06,
    10548.08,
    11175.3,
    11839.82,
    12543.85,
};

// 简谱音级到半音偏移量（大调）
const int SCALE_OFFSET[] = {0, 2, 4, 5, 7, 9, 11};

// 简谱解析函数
static int parse_jianpu(int base, const char *note)
{
    int dots_below = 0;
    int dots_above = 0;
    int note_num = 0;
    
    // 解析八度点
    const char* p = note;
    while (*p == '.') {
        dots_below++;
        p++;
    }
    
    // 解析音符数字
    note_num = *p - '0';
    p++;
    
    // 解析上方点
    while (*p == '.') {
        dots_above++;
        p++;
    }
    
    // 计算MIDI编号
    int octave_shift = dots_above - dots_below;
    int midi = base + SCALE_OFFSET[note_num - 1] + octave_shift * 12;
    
    // 计算时值（示例：假设4分音符为480 ticks）
    // int duration = 480; // 默认四分音符
    
    return midi;
}


static int convert_note_to_pos(char *note)
{
    // note-->midi
    int midi = parse_jianpu(40, note);
    //midi-->frequency
    double freq = g_midi_freq[midi];
    //frequency-->length
    double length = sqrt(k / freq);
    ESP_LOGI(TAG, "midi =%d, freq = %f, length = %f", midi, freq, length);
    //length-->pos
    if (length < LENGHT_MIN) {
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    int pos = (int)((length - LENGHT_MIN) / TOTAL_LENGTH * TOTAL_POS);

    return pos;
}


