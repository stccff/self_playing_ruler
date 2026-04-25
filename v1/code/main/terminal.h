#ifndef __TERMINAL_H
#define __TERMINAL_H

#include <stdbool.h>
#include <stddef.h>

/** Initialize the terminal engine and register all commands.
 *  Must be called once before terminal_feed() or terminal_prompt().
 *  @param  history_max   max history entries (1–256)
 */
void terminal_init(size_t history_max);

/** Feed one raw byte to the terminal state machine.
 *  Processes printable characters, backspace, Delete, Enter,
 *  and ANSI escape sequences (arrow keys).
 *  @param  c   the byte to process
 *  @return true if a complete command was just executed
 */
bool terminal_feed(char c);

/** Print the prompt string ("> "). Call once after terminal_init(),
 *  before entering the read loop. */
void terminal_prompt(void);

#endif
