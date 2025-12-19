#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "ruler.h"
#include "uart_ctrl.h"
#include "math.h"
#include "note_decode.h"
#include "string.h"
#include "stdlib.h"
#include "esp_timer.h"
#include "h_bridge.h"
#include "play.h"
#include "tinyusb_device.h"
#include "rgb_led.h"

// uart configurations
#define ECHO_TEST_TXD (43)
#define ECHO_TEST_RXD (44)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (0)
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (4096)


static const char *TAG = "UART_CTRL";




/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TERMINAL_BUFF_SIZE (1024)
#define CMD_MAX_LEN (32)
/* ***************************************************************************************************************** */
/*                                                type define                                                        */
/* ***************************************************************************************************************** */
typedef int (*cmd_cb_t)(char*);
/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */
typedef struct {
    char *cmd;
    cmd_cb_t func;
    char *param_info;
    char *help_info;
} cmd_table_t;

/* ***************************************************************************************************************** */
/*                                             function prototype                                                    */
/* ***************************************************************************************************************** */
static int do_cmd_help(char *data);
static int do_cmd_init_freq_table(char *data);
static int do_cmd_clear_freq_table(char *data);
static int do_cmd_freq_table_show(char *data);
static int do_cmd_set_midi_chan(char *data);
static int do_cmd_set(char *data);
static int do_cmd_play_by_note(char *data);
static int do_cmd_play_by_len(char *data);
static int do_cmd_play_by_pos(char *data);
static int do_cmd_testmagnet(char *data);
static int do_cmd_testmagnet2(char *data);
static int do_cmd_servo(char *data);
static int do_cmd_servo_offset(char *data);
static int do_cmd_stepper_motor(char *data);
static int do_cmd_enable_midi_velocity(char *data);
static int do_cmd_set_use_formula(char *data);
static int do_cmd_recalculate_params(char *data);
static int do_cmd_pitch_test(char *data);
static int do_cmd_servo_cali(char *data);
static int do_cmd_led_hsv(char *data);
static int do_cmd_print_compile_time(char *data);
/* ***************************************************************************************************************** */
/*                                              global variable                                                      */
/* ***************************************************************************************************************** */
cmd_table_t g_cmd_table[] = {
    {"help", do_cmd_help, "", ""},
    /* legacy prototype compatible commands */
    {"set", do_cmd_set, "<base> <scale>", "base: 'do' in midi, scale: musical mode"},
    {"p", do_cmd_play_by_note, "<note>", "play the note, eg.'#2.' means high re sharp"},
    /* latest */
    {"ftinit", do_cmd_init_freq_table, "", "init frequency table"},
    {"ftclear", do_cmd_clear_freq_table, "", "clear frequency table"},
    {"ftshow", do_cmd_freq_table_show, "", "print frequency table"},
    {"midich", do_cmd_set_midi_chan, "<ch>", "set actived midi input channel"},
    {"playlen", do_cmd_play_by_len, "<len>", "ruler play by length"},
    {"playpos", do_cmd_play_by_pos, "<pos>", "ruler play by stepper motor absolute position"},
    {"magnet", do_cmd_testmagnet, "<idx> <polary>", "set e-magnet"},
    {"magnet2", do_cmd_testmagnet2, "<idx1> <polary1> <idx2> <polary2>", "set 2 e-magnets at onece"},
    {"servoangle", do_cmd_servo, "<index> <angle>", "set servo motor angle"},
    {"servooffset", do_cmd_servo_offset, "<index> <angle>", "set servo motor offset angle"},
    {"stepper", do_cmd_stepper_motor, "<step>", "set stepper motor positon"},
    {"midivelocity", do_cmd_enable_midi_velocity, "<enable>", "enable or disable midi velocity, 0: disable, 1: enable"},
    {"useformula", do_cmd_set_use_formula, "<enable>", "use formula to calculate pos by freq, 0: use table, 1: use formula"},
    {"recalcparam", do_cmd_recalculate_params, "", "recalculate formula's parameters by current frequency table"},
    {"pitchtest", do_cmd_pitch_test, "", "test pitch accuracy"},
    {"servocali", do_cmd_servo_cali, "", "calibrate strum servo's middle angle"},
    {"ledhsv", do_cmd_led_hsv, "<h(0~360)> <s(0~1.0)> <v(0~1.0)>", "test rgb led by hsv"},
    {"compiletime", do_cmd_print_compile_time, "", "print the code compile time"},
};

static int do_cmd_help(char *data)
{
    printf("Usage: cmd [param]...\n");
    printf("List of cmd:\n");
    for (int i = 0; i < sizeof(g_cmd_table) / sizeof(g_cmd_table[0]); i++) {
        printf("%s %s:\n\t%s\n", g_cmd_table[i].cmd, g_cmd_table[i].param_info, g_cmd_table[i].help_info);
    }
    return ESP_OK;
}


static int do_cmd_set(char *data)
{
    int base, scale;
    if (sscanf(data, "%d %d", &base, &scale) != 2) {
        ESP_LOGE(TAG, "Invalid set command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }
    return set_base_and_scale(base, scale);
}


#define POLARITY_POSITIVE 1
#define POLARITY_NEGATIVE -1

static int do_cmd_play_by_note(char *data)
{
    int midi = parse_simple_note_to_midi(data);
    float freq = convert_midi_to_freq(midi);
    // ESP_LOGI(TAG, "midi = %d, freq = %f", midi, freq);

    return play_single_note_by_freq(freq);
}

static int do_cmd_play_by_len(char *data)
{
    double len = 0;
    int rc = sscanf(data, "%lf\n", &len);
    if (rc != 1) {
        ESP_LOGE(TAG, "Invalid test command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    return play_single_note_by_len(len);
}

static int do_cmd_play_by_pos(char *data)
{
    int pos = 0;
    int rc = sscanf(data, "%d\n", &pos);
    if (rc != 1) {
        ESP_LOGE(TAG, "Invalid test command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    return play_single_note_by_pos(pos);
}

static int do_cmd_init_freq_table(char *data)
{
    int rc = freq_table_init(true); // force init
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create frequency table");
        return rc;
    }
    ESP_LOGI(TAG, "Frequency table initialized successfully");
    return ESP_OK;
}

static int do_cmd_clear_freq_table(char *data)
{
    int rc = freq_table_clear();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear frequency table");
        return rc;
    }
    ESP_LOGI(TAG, "Frequency table clear success");
    return ESP_OK;
}

static int do_cmd_freq_table_show(char *data)
{
    int rc = freq_table_show();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to show frequency table");
        return rc;
    }
    ESP_LOGI(TAG, "Frequency table show success");
    return ESP_OK;
}

static int do_cmd_set_midi_chan(char *data)
{
    int ch = 0;
    int rc = sscanf(data, "%d\n", &ch);
    if (rc != 1) {
        ESP_LOGE(TAG, "Invalid set channel command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }
    midi_set_input_channel(ch);
    return ESP_OK;
}

static int do_cmd_testmagnet(char *data)
{
    int index = 0;
    int polary = 0;
    int rc = sscanf(data, "%d %d\n", &index, &polary);
    if (rc != 2) {
        ESP_LOGE(TAG, "Invalid test magnet command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    h_bridge_set(index, polary);

    return ESP_OK;
}

static int do_cmd_testmagnet2(char *data)
{
    int index = 0;
    int polary = 0;
    int index2 = 0;
    int polary2 = 0;
    int rc = sscanf(data, "%d %d %d %d\n", &index, &polary, &index2, &polary2);
    if (rc != 4) {
        ESP_LOGE(TAG, "Invalid test magnet command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    h_bridge_set(index, polary);
    h_bridge_set(index2, polary2);

    return ESP_OK;
}

static int do_cmd_servo(char *data)
{
    int index = 0;
    int angle = 0;
    int rc = sscanf(data, "%d %d\n", &index, &angle);
    if (rc != 2) {
        ESP_LOGE(TAG, "Invalid test servo command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }
    return servo_set_angle(index, angle);
}

static int do_cmd_servo_offset(char *data)
{
    int index = 0;
    int angle = 0;
    int rc = sscanf(data, "%d %d\n", &index, &angle);
    if (rc != 2) {
        ESP_LOGE(TAG, "Invalid servo offset command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }
    return servo_set_offset_angle(index, angle);
}

static int do_cmd_stepper_motor(char *data)
{
    int step = 0;
    int rc = sscanf(data, "%d\n", &step);
    if (rc != 1) {
        ESP_LOGE(TAG, "Invalid stepper motor command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    h_bridge_set(0, 0);
    h_bridge_set(1, 0);

    return stepper_motor_action_by_pos(true, step);
}

static int do_cmd_enable_midi_velocity(char *data)
{
    int velocity = 0;
    int rc = sscanf(data, "%d\n", &velocity);
    if (rc != 1) {
        ESP_LOGE(TAG, "Invalid midi velocity command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    midi_enable_volecity(velocity != 0);

    return ESP_OK;
}

static int do_cmd_set_use_formula(char *data)
{
    int enable = 0;
    int rc = sscanf(data, "%d\n", &enable);
    if (rc != 1) {
        ESP_LOGE(TAG, "Invalid useformula command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    freq_table_use_formula(enable != 0);
    if (enable != 0) {
        ESP_LOGI(TAG, "Using formula to calculate pos by freq");
    } else {
        ESP_LOGI(TAG, "Using table to calculate pos by freq");
    }

    return ESP_OK;
}

static int do_cmd_recalculate_params(char *data)
{
    int rc = recalculate_params();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to recalculate k and b");
        return rc;
    }
    ESP_LOGI(TAG, "Recalculate parameters success");
    return ESP_OK;
}

static int do_cmd_pitch_test(char *data)
{
    int rc = pitch_accuracy_test();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to test pitch accuracy");
        return rc;
    }
    ESP_LOGI(TAG, "Pitch accuracy test done");
    return ESP_OK;
}

static int do_cmd_servo_cali(char *data)
{
    servo_offset_calibration();
    return ESP_OK;
}

static int do_cmd_led_hsv(char *data)
{
    int rc = ESP_OK;
    int h;
    float s, v;

    rc = sscanf(data, "%d %f %f\n", &h, &s, &v);
    if (rc != 3) {
        ESP_LOGE(TAG, "Invalid ledhsv command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }

    rc = devkitc_rgb_led_set_hsv(h, s, v);

    return rc;
}

static int do_cmd_print_compile_time(char *data)
{
    ESP_LOGI(TAG, "Compile time: %s %s", __DATE__, __TIME__);
    return ESP_OK;
}


/**
 * @brief
 *
 * @param command
 * @return int
 */
static int dispatch_uart_cmd(char *command)
{
    int rc = ESP_OK;

    char cmdbuff[CMD_MAX_LEN];
    rc = sscanf(command, "%s", cmdbuff);
    if (rc != 1) {
        goto err;
    }
    for (int i = 0; i < sizeof(g_cmd_table) / sizeof(g_cmd_table[0]); i++) {
        if (strcmp(cmdbuff, g_cmd_table[i].cmd) == 0) {
            char *data = strstr(command, cmdbuff) + strlen(cmdbuff) + 1; // remove all space before 'data'
            rc = g_cmd_table[i].func(data);
            return rc;
        }
    }

err:
    ESP_LOGW(TAG, "Invalid command: %s", command);

    return ESP_OK;
}

/**
 * @brief a uart terminal TODO: need reconstruction(when two cmd's interval <10ms, may lost 2nd cmd, becasue 1st's CR/LF)
 *
 * @param cmd_parse_func
 * @param loop_delay
 */
static void start_terminal(int (*cmd_parse_func)(char*),uint32_t loop_delay)
{
    char *data = (char *) malloc(TERMINAL_BUFF_SIZE);

    int total_len = 0;
    uint32_t no_data_num = 0;
    bool is_end = false;
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, &data[total_len], (TERMINAL_BUFF_SIZE - total_len - 2), loop_delay / portTICK_PERIOD_MS);
        if (len > 0) {
            while (len > 0) {
                if (data[total_len] == '\b') {
                    if (total_len == 0) {
                        len--;
                        memmove(&data[total_len], &data[total_len + 1], len);
                    } else {
                        total_len--;
                        len--;
                        putchar('\b'); // backspace
                        putchar(' '); // clear the character
                        putchar('\b'); // back to the position
                        memmove(&data[total_len], &data[total_len + 2], len);
                    }
                } else if (data[total_len] == '\r' || data[total_len] == '\n') { // cmd end flag
                    data[total_len] = '\0';
                    total_len = 0;
                    putchar('\n');
                    is_end = true;
                    break;
                } else {
                    putchar(data[total_len]);
                    total_len++;
                    len--;
                }
            }
            fflush(stdout);
            if (is_end) {
                is_end = false;
            } else {
                continue;
            }
            no_data_num = 0;
        } else { // timeout
            no_data_num++;
            continue;
        }
        // ESP_LOGI(TAG, "data = %s", data);

        int64_t start = esp_timer_get_time();
        (void)cmd_parse_func(data);
        int64_t end = esp_timer_get_time();
        ESP_LOGI(TAG, "UART command execution time: %lld ms", (end - start) / 1000);
    }
}

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

    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50)); // wait ESP_LOG print done

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, TERMINAL_BUFF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    start_terminal(dispatch_uart_cmd, 10); // 10 ms delay for reading data
}

void uart_init(void)
{
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 5, NULL);
}
