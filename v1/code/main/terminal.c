/**
 * @file terminal.c
 * @brief  Generic serial-terminal REPL engine with line editing and command history.
 *
 * Architecture:
 *   terminal_feed() receives one byte at a time from the UART read loop.
 *   It runs a minimal state machine that recognises:
 *     - printable ASCII (0x20–0x7e): insert at cursor
 *     - backspace (0x08 / 0x7f): delete before cursor
 *     - Enter (0x0d / 0x0a): commit line, dispatch command, add to history
 *     - ANSI escape sequences for arrow keys and Delete
 *
 *   Cursor movement and line redraw use ANSI control codes:
 *     \\r         carriage return (go to column 0)
 *     \\x1b[K     clear to end of line
 *     \\x1b[nD    cursor back n columns
 *     \\x1b[D     cursor back 1 column
 *     \\x1b[C     cursor forward 1 column
 *
 *   The history is a fixed-capacity ring buffer allocated on the heap.
 *   The command table is a compile-time static array.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "servo_motor.h"
#include "step_motor.h"
#include "ruler.h"
#include "h_bridge.h"
#include "play.h"
#include "note_decode.h"
#include "tinyusb_device.h"
#include "rgb_led.h"
#include "terminal.h"

/* ***************************************************************************************************************** */
/*                                               macro define                                                        */
/* ***************************************************************************************************************** */
#define TAG "TERMINAL"

#define LINE_BUFSZ      128     /* max chars per input line (incl. NUL)  */
#define HISTORY_MAX     8       /* default history capacity              */
#define CMD_NAME_MAX    32      /* max length of command name            */
#define PARAM_INFO_MAX  64      /* max length of parameter hint          */
#define HELP_INFO_MAX   128     /* max length of help description        */

#define CSI_PREFIX      "\x1b[" /* Control Sequence Introducer          */

/* ***************************************************************************************************************** */
/*                                                type define                                                        */
/* ***************************************************************************************************************** */

/** Callback type for command handlers.
 *  @param  args  argument string (whitespace-trimmed tail of the input)
 *  @return ESP_OK on success, or ESP_ERR_* error code
 */
typedef int (*cmd_cb_t)(char *args);

/** States of the ANSI escape-sequence parser inside terminal_feed().   */
typedef enum {
    STATE_NORMAL,       /* waiting for next byte                        */
    STATE_ESC,          /* saw \\x1b, waiting for [                     */
    STATE_CSI,          /* saw \\x1b[, waiting for final byte           */
    STATE_CSI_DEL,      /* saw \\x1b[3, waiting for ~                   */
} ansi_state_t;

/* ***************************************************************************************************************** */
/*                                               struct define                                                       */
/* ***************************************************************************************************************** */

/** One entry in the static command lookup table.                         */
typedef struct {
    char        name[CMD_NAME_MAX];
    cmd_cb_t    func;
    char        param_info[PARAM_INFO_MAX];
    char        help_info[HELP_INFO_MAX];
} cmd_entry_t;

/* ***************************************************************************************************************** */
/*                                           function prototype                                                      */
/* ***************************************************************************************************************** */
static int do_cmd_help(char *data);
static int do_cmd_history(char *data);
static int do_cmd_set(char *data);
static int do_cmd_play_by_note(char *data);
static int do_cmd_play_by_len(char *data);
static int do_cmd_play_by_pos(char *data);
static int do_cmd_init_freq_table(char *data);
static int do_cmd_clear_freq_table(char *data);
static int do_cmd_freq_table_show(char *data);
static int do_cmd_set_midi_chan(char *data);
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

/* ── line editing ─────────────────────────────────────────────────── */
static char  g_line[LINE_BUFSZ];     /** current input line             */
static int   g_cursor;               /** cursor position (0..len)       */
static char  g_draft[LINE_BUFSZ];    /** saved draft while browsing     */
static bool  g_has_draft;            /** true if draft holds content    */

/* ── history ring buffer ──────────────────────────────────────────── */
static char (*g_history)[LINE_BUFSZ]; /** ring buffer (heap)             */
static int   g_history_max;           /** capacity (entries)             */
static int   g_history_count;         /** current entry count            */
static int   g_history_head;          /** where next entry is written    */
static int   g_history_idx;           /** -1 = new input, 0..count-1     */

/* ── ANSI state machine ───────────────────────────────────────────── */
static ansi_state_t g_ansi = STATE_NORMAL;

/* ── command table (compile-time) ─────────────────────────────────── */
static cmd_entry_t g_cmds[] = {
    /* built-in */
    {"help", do_cmd_help, "", ""},
    {"history", do_cmd_history, "", ""},
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
/* stringify helper — used inside sscanf format strings */
#define STR_(x)  #x
#define STR(x)   STR_(x)

#define CMD_COUNT (sizeof(g_cmds) / sizeof(g_cmds[0]))

/* ***************************************************************************************************************** */
/*                                           internal helpers                                                         */
/* ***************************************************************************************************************** */

/**
 * @brief Repaint the entire input line from column 0.
 *
 * Sequence: \\r (carriage return) → "> " (prompt) → line text
 *           → \\x1b[K (clear to EOL) → move cursor back to position.
 */
static void terminal_redraw(void)
{
    int len = (int)strlen(g_line);
    printf("\r> %s" "\x1b[K", g_line);
    if (g_cursor < len) {
        /* cursor is currently at EOL after printing; pull it back */
        printf("\x1b[%dD", len - g_cursor);
    }
    fflush(stdout);
}

/**
 * @brief Insert character c at the cursor position, shifting tail right.
 */
static void terminal_insert_char(char c)
{
    int len = (int)strlen(g_line);
    if (len >= LINE_BUFSZ - 1) {
        return;
    }
    memmove(&g_line[g_cursor + 1], &g_line[g_cursor], len - g_cursor + 1);
    g_line[g_cursor] = c;
    g_cursor++;
    terminal_redraw();
}

/**
 * @brief Delete the character before the cursor (backspace).
 */
static void terminal_backspace(void)
{
    int len = (int)strlen(g_line);
    if (g_cursor <= 0) {
        return;
    }
    memmove(&g_line[g_cursor - 1], &g_line[g_cursor], len - g_cursor + 1);
    g_cursor--;
    terminal_redraw();
}

/**
 * @brief Delete the character after the cursor (Delete key, \\e[3~).
 */
static void terminal_delete_forward(void)
{
    int len = (int)strlen(g_line);
    if (g_cursor >= len) {
        return;
    }
    memmove(&g_line[g_cursor], &g_line[g_cursor + 1], len - g_cursor);
    terminal_redraw();
}

/**
 * @brief Move cursor one column left.
 */
static void terminal_cursor_left(void)
{
    if (g_cursor > 0) {
        g_cursor--;
        printf("\x1b[D");
        fflush(stdout);
    }
}

/**
 * @brief Move cursor one column right.
 */
static void terminal_cursor_right(void)
{
    int len = (int)strlen(g_line);
    if (g_cursor < len) {
        g_cursor++;
        printf("\x1b[C");
        fflush(stdout);
    }
}

/* ── history ring buffer ──────────────────────────────────────────── */

/**
 * @brief Push a command into the ring buffer.
 *
 * Empty strings and consecutive duplicates are silently dropped.
 */
static void terminal_history_add(const char *cmd)
{
    if (cmd[0] == '\0') {
        return;
    }
    /* skip if identical to the most recent entry */
    if (g_history_count > 0) {
        int last_idx = (g_history_head - 1 + g_history_max) % g_history_max;
        if (strcmp(g_history[last_idx], cmd) == 0) {
            return;
        }
    }

    strncpy(g_history[g_history_head], cmd, LINE_BUFSZ);
    g_history[g_history_head][LINE_BUFSZ - 1] = '\0';
    g_history_head = (g_history_head + 1) % g_history_max;
    if (g_history_count < g_history_max) {
        g_history_count++;
    }
}

/**
 * @brief Recall the previous (older) history entry.
 *
 * On first invocation the current line is saved as a draft so that
 * terminal_history_down() can restore it later.
 */
static void terminal_history_up(void)
{
    if (g_history_count == 0) {
        return;
    }

    if (g_history_idx < 0) {
        /* save draft on first press */
        strncpy(g_draft, g_line, LINE_BUFSZ);
        g_has_draft = true;
        g_history_idx = g_history_count - 1;
    } else if (g_history_idx > 0) {
        g_history_idx--;
    } else {
        return;  /* already at oldest entry */
    }

    int idx = (g_history_head - g_history_count + g_history_idx
               + g_history_max) % g_history_max;
    strncpy(g_line, g_history[idx], LINE_BUFSZ);
    g_line[LINE_BUFSZ - 1] = '\0';
    g_cursor = (int)strlen(g_line);
    terminal_redraw();
}

/**
 * @brief Recall the next (newer) history entry.
 *
 * Past the newest entry the saved draft is restored.
 */
static void terminal_history_down(void)
{
    if (g_history_idx < 0) {
        return;
    }

    if (g_history_idx < g_history_count - 1) {
        g_history_idx++;
        int idx = (g_history_head - g_history_count + g_history_idx
                   + g_history_max) % g_history_max;
        strncpy(g_line, g_history[idx], LINE_BUFSZ);
        g_line[LINE_BUFSZ - 1] = '\0';
    } else {
        /* past newest → restore the saved draft */
        g_history_idx = -1;
        if (g_has_draft) {
            strncpy(g_line, g_draft, LINE_BUFSZ);
            g_line[LINE_BUFSZ - 1] = '\0';
            g_has_draft = false;
        } else {
            g_line[0] = '\0';
        }
    }
    g_cursor = (int)strlen(g_line);
    terminal_redraw();
}

/* ── command dispatch ─────────────────────────────────────────────── */

/**
 * @brief Look up @p cmdline in the command table and invoke the handler.
 *
 * The command name is the first whitespace-delimited token.
 * Arguments (the rest of the line) are passed to the handler.
 */
static int dispatch(const char *cmdline)
{
    char name[CMD_NAME_MAX + 1] = {0};
    sscanf(cmdline, "%" STR(CMD_NAME_MAX) "s", name);
    if (name[0] == '\0') {
        return 0;
    }

    for (int i = 0; i < (int)CMD_COUNT; i++) {
        if (strcmp(name, g_cmds[i].name) == 0) {
            const char *args = cmdline + strlen(name);
            while (*args == ' ') {
                args++;
            }
            return g_cmds[i].func((char *)args);
        }
    }

    ESP_LOGW(TAG, "Invalid command: %s", name);
    return 0;
}

/**
 * @brief Called when Enter is pressed.
 *
 * Commits the line, dispatches it, adds to history, and resets
 * the editing state for the next input.
 */
static void terminal_newline(void)
{
    printf("\n");

    terminal_history_add(g_line);
    dispatch(g_line);

    g_line[0] = '\0';
    g_cursor = 0;
    g_history_idx = -1;
    g_has_draft = false;
    terminal_prompt();
}

/* ***************************************************************************************************************** */
/*                                           business commands                                                        */
/* ***************************************************************************************************************** */

/**
 * @brief Print usage info for all registered commands.
 */
static int do_cmd_help(char *data)
{
    printf("Usage: cmd [param]...\n");
    printf("List of cmd:\n");
    for (int i = 0; i < (int)CMD_COUNT; i++) {
        printf("%s %s:\n\t%s\n",
               g_cmds[i].name, g_cmds[i].param_info, g_cmds[i].help_info);
    }
    return ESP_OK;
}

/**
 * @brief Print all entries in the command history.
 */
static int do_cmd_history(char *data)
{
    for (int i = 0; i < g_history_count; i++) {
        int idx = (g_history_head - g_history_count + i
                   + g_history_max) % g_history_max;
        printf("  %d  %s\n", i + 1, g_history[idx]);
    }
    return 0;
}

/**
 * @brief Set musical base (MIDI number) and scale index.
 */
static int do_cmd_set(char *data)
{
    int base, scale;
    if (sscanf(data, "%d %d", &base, &scale) != 2) {
        ESP_LOGE(TAG, "Invalid set command param: %s", data);
        return ESP_ERR_INVALID_ARG;
    }
    return set_base_and_scale(base, scale);
}


#define POLARITY_POSITIVE  1
#define POLARITY_NEGATIVE -1

/**
 * @brief Play a single note by simplified (jianpu) notation.
 */
static int do_cmd_play_by_note(char *data)
{
    int midi = parse_simple_note_to_midi(data);
    float freq = convert_midi_to_freq(midi);

    return play_single_note_by_freq(freq);
}

/**
 * @brief Play a single note by ruler length (mm).
 */
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

/**
 * @brief Play a single note by stepper motor absolute position.
 */
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

/**
 * @brief Force recreation of the frequency-to-position calibration table.
 */
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

/**
 * @brief Clear the calibration table from NVS.
 */
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

/**
 * @brief Dump the current frequency-to-position table to stdout.
 */
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

/**
 * @brief Set the active MIDI input channel.
 */
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

/**
 * @brief Manually control a single electromagnet.
 */
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

/**
 * @brief Manually control two electromagnets at once.
 */
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

/**
 * @brief Set servo angle by index.
 */
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

/**
 * @brief Set servo offset angle and persist to NVS.
 */
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

/**
 * @brief Move the stepper motor to an absolute step position.
 */
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

/**
 * @brief Enable or disable MIDI velocity processing.
 */
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

/**
 * @brief Toggle between formula-based and table-based frequency lookup.
 */
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

/**
 * @brief Recalculate the formula parameters (k, a, b) from the current
 *        frequency table using Levenberg-Marquardt fitting.
 */
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

/**
 * @brief Run a pitch accuracy test across the MIDI range of the ruler.
 */
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

/**
 * @brief Run servo strum offset calibration using microphone feedback.
 */
static int do_cmd_servo_cali(char *data)
{
    servo_offset_calibration();
    return ESP_OK;
}

/**
 * @brief Set the RGB LED colour using HSV values.
 */
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

/**
 * @brief Print the firmware compilation timestamp and IDF version.
 */
static int do_cmd_print_compile_time(char *data)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();

    ESP_LOGI(TAG, "Firmware Compile time: %s %s", app_desc->date, app_desc->time);
    ESP_LOGI(TAG, "Firmware Version: %s", app_desc->version);
    ESP_LOGI(TAG, "IDF Version: %s", app_desc->idf_ver);

    return ESP_OK;
}

/* ***************************************************************************************************************** */
/*                                              public API                                                            */
/* ***************************************************************************************************************** */

void terminal_init(size_t history_max)
{
    if (history_max > 256) {
        history_max = 256;
    }

    /* allocate the history ring buffer */
    g_history = (char (*)[LINE_BUFSZ])calloc(history_max, LINE_BUFSZ);
    if (g_history == NULL) {
        ESP_LOGE(TAG, "Failed to allocate history buffer");
        return;
    }
    g_history_max = (int)history_max;
    g_history_count = 0;
    g_history_head = 0;
    g_history_idx = -1;

    /* init line-editing state */
    g_cursor = 0;
    g_has_draft = false;
    g_line[0] = '\0';

    /* init ANSI state machine */
    g_ansi = STATE_NORMAL;
}

void terminal_prompt(void)
{
    printf("> ");
    fflush(stdout);
}

bool terminal_feed(char c)
{
    switch (g_ansi) {

    case STATE_NORMAL:
        if (c == '\x1b') {
            g_ansi = STATE_ESC;
        } else if (c == '\r' || c == '\n') {
            terminal_newline();
            return true;
        } else if (c == '\b' || c == 0x7f) {
            terminal_backspace();
        } else if (c >= 0x20 && c < 0x7f) {
            terminal_insert_char(c);
        }
        /* other control characters (0x00–0x1f) are silently ignored */
        break;

    case STATE_ESC:
        /* expect '[' after ESC for CSI sequences; anything else → ignore */
        if (c == '[') {
            g_ansi = STATE_CSI;
        } else {
            g_ansi = STATE_NORMAL;
        }
        break;

    case STATE_CSI:
        switch (c) {
        case 'A': terminal_history_up();    break;   /* ↑                 */
        case 'B': terminal_history_down();  break;   /* ↓                 */
        case 'C': terminal_cursor_right();  break;   /* →                 */
        case 'D': terminal_cursor_left();   break;   /* ←                 */
        case '3': g_ansi = STATE_CSI_DEL;   return false;  /* wait for ~ */
        default:
            /* unknown CSI sequence — ignore */
            break;
        }
        g_ansi = STATE_NORMAL;
        break;

    case STATE_CSI_DEL:
        /* Delete key: \\e[3~ — delete character after cursor */
        if (c == '~') {
            terminal_delete_forward();
        }
        g_ansi = STATE_NORMAL;
        break;
    }

    return false;
}
