/*
 * This file is part of DeskHop (https://github.com/hrvach/deskhop).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * See the file LICENSE for the full license text.
 */

/* Transparent passthrough of an Apple Magic Keyboard with Touch ID.
 *
 * When such a keyboard is plugged into this board's host port, the board stops
 * presenting itself as "DeskHop Switch" and re-enumerates as an exact copy of
 * the keyboard (descriptors, strings, all HID interfaces). Interrupt IN data and
 * HID control requests are relayed both ways, so the computer talks to the real
 * keyboard, including its Touch ID sensor. DeskHop keeps parsing the keyboard
 * interface so hotkeys and forwarding to the other board still work. */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Flags returned by pt_host_report() */
#define PT_RX_REARM 0x01 /* Caller should request the next report now */
#define PT_RX_PARSE 0x02 /* Caller should run the normal DeskHop report parser */
#define PT_RX_HELD  0x04 /* Not relayed yet: call pt_host_report_finish() after parsing */

#define PT_NO_IDX 0xFF

#ifdef DH_PASSTHROUGH

void pt_init(void);

/* Tasks: device side runs on core0, host side on core1 */
void pt_device_task(device_t *state);
void pt_host_task(device_t *state);

/* True while this board is enumerated (or enumerating) as the cloned keyboard. */
bool pt_clone_active(void);

/* True if dev_addr is the keyboard being passed through. */
bool pt_is_target(uint8_t dev_addr);

/* Host side hooks (core1) */
void    pt_host_hid_mount(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len);
void    pt_host_umount(uint8_t dev_addr);
uint8_t pt_host_report(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len);
uint8_t pt_host_report_finish(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len, bool understood);

/* Bracket the DeskHop parser while it looks at a report of the passed-through keyboard */
void pt_parse_begin(uint8_t device_idx);
void pt_parse_end(void);
bool pt_parsing_target(void);

/* Parser verdict: this report was a DeskHop hotkey, the computer must not see it */
void pt_kbd_swallow(void);

/* Keyboard state slot to leave out of reports on DeskHop's own keyboard interface, because
   that keyboard already reaches this computer directly. PT_NO_IDX if there is none. */
uint8_t pt_local_skip_idx(void);

/* Cached descriptors of the cloned keyboard, valid while pt_clone_active() */
uint8_t const  *pt_desc_device(void);
uint8_t const  *pt_desc_configuration(void);
uint16_t const *pt_desc_string(uint8_t index);

#else

static inline void pt_init(void) {}
static inline bool pt_clone_active(void) { return false; }
static inline bool pt_is_target(uint8_t dev_addr) { (void)dev_addr; return false; }
static inline void pt_host_hid_mount(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
    (void)dev_addr; (void)instance; (void)desc_report; (void)desc_len;
}
static inline void pt_host_umount(uint8_t dev_addr) { (void)dev_addr; }
static inline uint8_t pt_host_report(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
    (void)dev_addr; (void)instance; (void)report; (void)len;
    return PT_RX_REARM | PT_RX_PARSE;
}
static inline uint8_t pt_host_report_finish(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len, bool understood) {
    (void)dev_addr; (void)instance; (void)report; (void)len; (void)understood;
    return PT_RX_REARM;
}
static inline void    pt_parse_begin(uint8_t device_idx) { (void)device_idx; }
static inline void    pt_parse_end(void) {}
static inline bool    pt_parsing_target(void) { return false; }
static inline void    pt_kbd_swallow(void) {}
static inline uint8_t pt_local_skip_idx(void) { return PT_NO_IDX; }
static inline uint8_t const  *pt_desc_device(void) { return NULL; }
static inline uint8_t const  *pt_desc_configuration(void) { return NULL; }
static inline uint16_t const *pt_desc_string(uint8_t index) { (void)index; return NULL; }

#endif
