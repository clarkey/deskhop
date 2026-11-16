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
 * See passthrough.h for the overview.
 *
 * Threading model: the USB host stack runs on core1, the USB device stack on
 * core0. Everything that crosses between them goes through pico queue_t (which
 * is multicore safe) or through single-writer volatile state variables.
 * Large buffers are static, both cores only have a 2 KB stack. */

#include "main.h"

#ifdef DH_PASSTHROUGH

#include "device/dcd.h"
#include "device/usbd_pvt.h"

/* ---------- Limits ---------- */
#define PT_MAX_ITF       4    /* HID interfaces we are willing to clone */
#define PT_CFG_MAX       256  /* Max configuration descriptor size */
#define PT_CFG_OUT_MAX   512  /* Keyboard's configuration plus DeskHop's own interfaces */
#define PT_GATED_IDS     4    /* Distinct key-type report IDs we can send a release for */
#define PT_RPT_MAX       512  /* Max HID report descriptor size, per interface */
#define PT_STR_SLOTS     8    /* Number of cached string descriptors (incl. langid) */
#define PT_STR_MAX       128  /* Max string descriptor size in bytes */
#define PT_CTRL_MAX      4096 /* Max control transfer data stage we relay (Touch ID uses up to 2564) */
#define PT_PKT_MAX       64   /* Full speed interrupt packet */
#define PT_IN_QUEUE_LEN  32   /* Packets buffered per interface, keyboard -> computer */

#define PT_SWITCH_DELAY_US  500000  /* Time we stay off the bus when re-enumerating */
#define PT_CAPTURE_LIMIT_US 3000000 /* Give up capturing descriptors after this long */
#define PT_CTRL_TIMEOUT_US  4000000 /* Refuse a relayed request the keyboard has not answered by then */

#define PT_NO_ITF 0xFF

/* Apple keyboards with Touch ID. Only 0x0322 has been tested. */
#define PT_VID 0x05ac
static const uint16_t pt_pids[] = {
    0x029c, /* Magic Keyboard with Touch ID (Lightning) */
    0x029f, /* Magic Keyboard with Touch ID and Numeric Keypad (Lightning) */
    0x0321, /* Magic Keyboard with Touch ID (USB-C) */
    0x0322, /* Magic Keyboard with Touch ID and Numeric Keypad (USB-C) */
};

/* ---------- Types ---------- */
typedef struct {
    uint8_t len;
    uint8_t data[PT_PKT_MAX];
} pt_pkt_t;

/* The one control request in flight between the two cores. The control endpoint is
   strictly serial (the computer waits for each answer), so a single slot is enough.
   The state byte hands ownership back and forth:
   X_FREE -> X_REQ (core0 filled it in) -> X_BUSY (core1 sent it) -> X_DONE (core1) -> X_FREE (core0) */
enum { X_FREE = 0, X_REQ, X_BUSY, X_DONE };

typedef struct {
    volatile uint8_t       state;
    uint8_t                seq; /* Lets core0 recognise an answer to a request it has given up on */
    uint8_t                ok;
    uint16_t               actual_len;
    tusb_control_request_t setup;
    CFG_TUSB_MEM_ALIGN uint8_t data[PT_CTRL_MAX];
} pt_xchg_t;

/* Device side progress of the request currently being relayed */
enum { C_IDLE = 0, C_RX_DATA, C_WAIT_SLOT, C_WAIT_HOST };

typedef struct {
    uint8_t  index;
    uint8_t  valid;
    uint16_t raw[PT_STR_MAX / 2]; /* Raw descriptor as sent by the keyboard */
} pt_str_t;

enum { HS_NONE = 0, HS_MOUNTING, HS_CAPTURE, HS_READY, HS_FAILED };
enum { DS_NORMAL = 0, DS_TO_CLONE, DS_CLONE, DS_TO_NORMAL };
enum { CAP_DEVICE = 0, CAP_CFG_HEADER, CAP_CFG_FULL, CAP_LANGID, CAP_STRINGS, CAP_DONE };

/* ---------- State ---------- */
static struct {
    /* --- Written by core1 (host side) --- */
    volatile uint8_t host_state;
    volatile uint8_t gen;  /* Bumped on every successful capture */
    uint8_t daddr;
    uint8_t fail_code;     /* Why the last capture was abandoned, for debugging */

    /* Captured descriptors. Written during capture only, read-only afterwards. */
    CFG_TUSB_MEM_ALIGN uint8_t dev_desc[sizeof(tusb_desc_device_t)];
    CFG_TUSB_MEM_ALIGN uint8_t cfg_desc[PT_CFG_MAX];
    CFG_TUSB_MEM_ALIGN uint8_t rpt_desc[PT_MAX_ITF][PT_RPT_MAX];
    uint16_t rpt_len[PT_MAX_ITF];
    uint16_t hid_desc_ofs[PT_MAX_ITF]; /* Offset of the HID descriptor inside cfg_desc */
    uint8_t  ep_mps[PT_MAX_ITF];       /* Packet size of the interface's IN endpoint */
    uint8_t  num_itf;
    uint8_t  kbd_itf;                  /* Interface carrying the actual key reports */
    pt_str_t str[PT_STR_SLOTS];
    uint16_t langid;

    /* Capture state machine */
    uint8_t  cap_step;
    uint8_t  cap_str_slot;
    bool     cap_busy;
    uint32_t cap_started;

    /* Report forwarding */
    bool    parsing;        /* DeskHop's parser is looking at one of the keyboard's reports */
    bool    swallow;        /* ... and decided it was a hotkey */
    uint8_t kbd_dev_idx;    /* Keyboard state slot DeskHop uses for the passed-through keyboard */
    bool    was_active;     /* This board was the active output on the previous pass */
    bool    drop_rest[CFG_TUH_HID]; /* Rest of a held-back report must be dropped as well */
    struct {
        uint8_t id;
        uint8_t len;
    } gated[PT_GATED_IDS];  /* Key-type reports seen, so we know how to say "all released" */
    uint8_t inst2itf[CFG_TUH_HID];
    bool    rearm_pending[CFG_TUH_HID];
    bool    rx_continues[CFG_TUH_HID]; /* Next packet is the middle of a long report */
    pt_pkt_t rx_pkt;

    /* Control relay, host side */
    bool     ctl_inflight;
    bool     ctl_host_waiting;
    uint32_t ctl_host_since;

    /* --- Written by core0 (device side) --- */
    volatile uint8_t dev_state;
    volatile bool    clone_active;
    CFG_TUSB_MEM_ALIGN uint8_t cfg_out[PT_CFG_OUT_MAX]; /* What we actually present */
    uint8_t  active_gen;
    uint32_t switch_started;

    uint8_t  ep_in[PT_MAX_ITF]; /* Opened IN endpoint per cloned interface, 0 if none */
    pt_pkt_t tx_pkt[PT_MAX_ITF];

    /* Control relay, device side */
    uint8_t                ctl_state;
    uint8_t                ctl_seq;
    uint32_t               ctl_since;
    tusb_control_request_t ctl_setup;
    CFG_TUSB_MEM_ALIGN uint8_t ctl_buf[PT_CTRL_MAX];

    /* --- Shared --- */
    queue_t   in_q[PT_MAX_ITF]; /* Interrupt/bulk IN packets, core1 -> core0 */
    pt_xchg_t xchg;             /* Control request in flight, see pt_xchg_t */
} pt;

/* ==================================================== *
 * Debug trace (DH_PASSTHROUGH_TRACE): every relayed control request is recorded
 * in RAM. A few seconds after becoming the clone, the recording is printed on
 * the debug serial port, once a second.
 * ==================================================== */
#ifdef DH_PASSTHROUGH_TRACE
#define PT_TRACE_LEN      200
#define PT_TRACE_CLONE_US 12000000

typedef struct {
    uint32_t t_ms;
    char     ev;
    uint8_t  bm, req;
    uint16_t val, idx, len, aux;
} pt_trace_t;

/* One log per core, so neither needs a lock */
static pt_trace_t        trace_log[2][PT_TRACE_LEN];
static volatile uint16_t trace_count[2];
static volatile uint32_t in_fwd[PT_MAX_ITF], in_drop[PT_MAX_ITF];
static volatile bool     trace_hold;

static void trace(uint8_t core, char ev, tusb_control_request_t const *r, uint16_t aux) {
    uint16_t n = trace_count[core];

    if (n >= PT_TRACE_LEN)
        return;

    pt_trace_t *t = &trace_log[core][n];
    t->t_ms = to_ms_since_boot(get_absolute_time());
    t->ev   = ev;
    t->aux  = aux;
    if (r) {
        t->bm  = r->bmRequestType;
        t->req = r->bRequest;
        t->val = r->wValue;
        t->idx = r->wIndex;
        t->len = r->wLength;
    } else {
        t->bm = t->req = 0;
        t->val = t->idx = t->len = 0;
    }
    trace_count[core] = n + 1;
}
#define TRACE_DEV(ev, r, aux)  trace(0, ev, r, aux)
#define TRACE_HOST(ev, r, aux) trace(1, ev, r, aux)
#define COUNT_IN(arr, itf)     ((arr)[itf]++)
#else
#define TRACE_DEV(ev, r, aux)  ((void)0)
#define TRACE_HOST(ev, r, aux) ((void)0)
#define COUNT_IN(arr, itf)     ((void)0)
#endif

/* ==================================================== *
 * Common
 * ==================================================== */

void pt_init(void) {
    for (int i = 0; i < PT_MAX_ITF; i++)
        queue_init(&pt.in_q[i], sizeof(pt_pkt_t), PT_IN_QUEUE_LEN);

    memset(pt.inst2itf, PT_NO_ITF, sizeof(pt.inst2itf));
}

bool pt_clone_active(void) {
    return pt.clone_active;
}

bool pt_is_target(uint8_t dev_addr) {
    return pt.host_state != HS_NONE && pt.daddr == dev_addr;
}

uint8_t const *pt_desc_device(void) {
    return pt.dev_desc;
}

uint8_t const *pt_desc_configuration(void) {
    return pt.cfg_out;
}

void pt_parse_begin(uint8_t device_idx) {
    pt.kbd_dev_idx = device_idx;
    pt.swallow     = false;
    pt.parsing     = true;
}

void pt_parse_end(void) {
    pt.parsing = false;
}

bool pt_parsing_target(void) {
    return pt.parsing && pt.clone_active;
}

void pt_kbd_swallow(void) {
    pt.swallow = true;
}

uint8_t pt_local_skip_idx(void) {
    return pt.clone_active ? pt.kbd_dev_idx : PT_NO_IDX;
}

uint16_t const *pt_desc_string(uint8_t index) {
    for (int i = 0; i < PT_STR_SLOTS; i++)
        if (pt.str[i].valid && pt.str[i].index == index)
            return pt.str[i].raw;

    return NULL;
}

/* ==================================================== *
 * Host side (core1): descriptor capture
 * ==================================================== */

static bool is_supported_keyboard(uint8_t dev_addr) {
    uint16_t vid, pid;

    if (!tuh_vid_pid_get(dev_addr, &vid, &pid) || vid != PT_VID)
        return false;

    for (unsigned i = 0; i < ARRAY_SIZE(pt_pids); i++)
        if (pt_pids[i] == pid)
            return true;

    return false;
}

static void host_reset(void) {
    memset(pt.inst2itf, PT_NO_ITF, sizeof(pt.inst2itf));
    memset(pt.rearm_pending, 0, sizeof(pt.rearm_pending));
    memset(pt.rx_continues, 0, sizeof(pt.rx_continues));
    memset(pt.drop_rest, 0, sizeof(pt.drop_rest));
    memset(pt.gated, 0, sizeof(pt.gated));
    pt.kbd_dev_idx = PT_NO_IDX;
    memset(pt.rpt_len, 0, sizeof(pt.rpt_len));

    pt.cap_busy         = false;
    pt.ctl_inflight     = false;
    pt.ctl_host_waiting = false;
}

/* Called for every HID interface of every device, before tuh_mount_cb. */
void pt_host_hid_mount(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
    tuh_itf_info_t info;

    /* One passthrough keyboard at a time, and never in config mode */
    if (global_state.config_mode_active || !is_supported_keyboard(dev_addr))
        return;

    if (pt.host_state != HS_NONE && pt.daddr != dev_addr)
        return;

    if (pt.host_state == HS_NONE) {
        host_reset();
        pt.daddr      = dev_addr;
        pt.host_state = HS_MOUNTING;
    }

    if (instance >= CFG_TUH_HID || !tuh_hid_itf_get_info(dev_addr, instance, &info))
        return;

    uint8_t itf = info.desc.bInterfaceNumber;

    /* An interface we can't describe faithfully simply stays unmapped, which
       makes the capture fail later and leaves DeskHop in its normal mode. */
    if (itf >= PT_MAX_ITF || desc_report == NULL || desc_len == 0 || desc_len > PT_RPT_MAX)
        return;

    memcpy(pt.rpt_desc[itf], desc_report, desc_len);
    pt.rpt_len[itf]       = desc_len;
    pt.inst2itf[instance] = itf;
}

/* Called once the whole device is configured */
void tuh_mount_cb(uint8_t dev_addr) {
    if (pt.host_state != HS_MOUNTING || pt.daddr != dev_addr)
        return;

    memset(pt.str, 0, sizeof(pt.str));
    pt.fail_code   = 0;
    pt.cap_step    = CAP_DEVICE;
    pt.cap_busy    = false;
    pt.cap_started = time_us_32();
    pt.host_state  = HS_CAPTURE;
}

void pt_host_umount(uint8_t dev_addr) {
    if (pt.host_state == HS_NONE || pt.daddr != dev_addr)
        return;

    pt.host_state = HS_NONE;
    host_reset();

    /* A request that was on its way to the keyboard will never be answered now */
    if (pt.xchg.state == X_REQ || pt.xchg.state == X_BUSY) {
        pt.xchg.ok    = 0;
        pt.xchg.state = X_DONE;
    }
}

static void add_string_slot(uint8_t index) {
    if (index == 0)
        return;

    for (int i = 1; i < PT_STR_SLOTS; i++) {
        if (pt.str[i].index == index)
            return;

        if (pt.str[i].index == 0) {
            pt.str[i].index = index;
            return;
        }
    }
}

/* Check the configuration is something we can clone and note where things are. */
static bool parse_fail(uint8_t code) {
    pt.fail_code = code;
    return false;
}

static bool parse_configuration(void) {
    tusb_desc_configuration_t const *cfg = (tusb_desc_configuration_t const *)pt.cfg_desc;
    tusb_desc_device_t const        *dev = (tusb_desc_device_t const *)pt.dev_desc;

    uint16_t total = tu_le16toh(cfg->wTotalLength);
    uint8_t const *p   = pt.cfg_desc + cfg->bLength;
    uint8_t const *end = pt.cfg_desc + total;
    uint8_t itf        = PT_NO_ITF;
    uint8_t seen       = 0;

    if (cfg->bNumInterfaces == 0 || cfg->bNumInterfaces > PT_MAX_ITF)
        return parse_fail(21);

    memset(pt.hid_desc_ofs, 0, sizeof(pt.hid_desc_ofs));
    memset(pt.ep_mps, 0, sizeof(pt.ep_mps));
    pt.kbd_itf = PT_NO_ITF;

    while (p + 2 <= end && tu_desc_len(p) >= 2 && p + tu_desc_len(p) <= end) {
        switch (tu_desc_type(p)) {
            case TUSB_DESC_INTERFACE: {
                tusb_desc_interface_t const *d = (tusb_desc_interface_t const *)p;

                if (d->bInterfaceClass != TUSB_CLASS_HID || d->bAlternateSetting != 0)
                    return parse_fail(22);

                itf = d->bInterfaceNumber;
                if (itf >= PT_MAX_ITF || pt.rpt_len[itf] == 0)
                    return parse_fail(23);

                if (d->bInterfaceProtocol == HID_ITF_PROTOCOL_KEYBOARD)
                    pt.kbd_itf = itf;

                add_string_slot(d->iInterface);
                seen++;
                break;
            }

            case HID_DESC_TYPE_HID:
                if (itf == PT_NO_ITF)
                    return parse_fail(24);
                pt.hid_desc_ofs[itf] = (uint16_t)(p - pt.cfg_desc);
                break;

            case TUSB_DESC_ENDPOINT: {
                tusb_desc_endpoint_t const *d = (tusb_desc_endpoint_t const *)p;

                /* The Touch ID interface uses a bulk IN endpoint, the others interrupt IN.
                   Both are relayed packet by packet in the same way. */
                if ((d->bmAttributes.xfer != TUSB_XFER_INTERRUPT && d->bmAttributes.xfer != TUSB_XFER_BULK) ||
                    tu_edpt_packet_size(d) > PT_PKT_MAX)
                    return parse_fail(25);

                if (itf != PT_NO_ITF && tu_edpt_dir(d->bEndpointAddress) == TUSB_DIR_IN)
                    pt.ep_mps[itf] = (uint8_t)tu_edpt_packet_size(d);
                break;
            }

            default:
                break;
        }
        p += tu_desc_len(p);
    }

    if (seen != cfg->bNumInterfaces)
        return parse_fail(26);

    for (uint8_t i = 0; i < cfg->bNumInterfaces; i++)
        if (pt.hid_desc_ofs[i] == 0)
            return parse_fail(27);

    pt.num_itf = cfg->bNumInterfaces;

    add_string_slot(dev->iManufacturer);
    add_string_slot(dev->iProduct);
    add_string_slot(dev->iSerialNumber);
    add_string_slot(cfg->iConfiguration);

    return true;
}

static void capture_cb(tuh_xfer_t *xfer) {
    bool ok = (xfer->result == XFER_RESULT_SUCCESS);

    pt.cap_busy = false;

    if (pt.host_state != HS_CAPTURE)
        return;

    switch (pt.cap_step) {
        case CAP_DEVICE:
            if (!ok || xfer->actual_len < sizeof(tusb_desc_device_t))
                goto fail;
            pt.cap_step = CAP_CFG_HEADER;
            break;

        case CAP_CFG_HEADER: {
            uint16_t total = tu_le16toh(((tusb_desc_configuration_t const *)pt.cfg_desc)->wTotalLength);
            if (!ok || total > PT_CFG_MAX || total < sizeof(tusb_desc_configuration_t))
                goto fail;
            pt.cap_step = CAP_CFG_FULL;
            break;
        }

        case CAP_CFG_FULL:
            if (!ok || !parse_configuration())
                goto fail;
            pt.cap_step = CAP_LANGID;
            break;

        case CAP_LANGID:
            /* Slot 0 holds the language ID list. A keyboard without strings is fine. */
            pt.langid = 0x0409;
            if (ok && xfer->actual_len >= 4) {
                pt.str[0].valid = 1;
                pt.langid       = pt.str[0].raw[1];
            }
            pt.cap_str_slot = 1;
            pt.cap_step     = CAP_STRINGS;
            break;

        case CAP_STRINGS: {
            pt_str_t *s      = &pt.str[pt.cap_str_slot];
            uint8_t  *raw    = (uint8_t *)s->raw;
            uint16_t  actual = xfer->actual_len;

            /* A missing string is not fatal, the computer just gets a STALL for it */
            if (ok && actual >= 2 && raw[1] == TUSB_DESC_STRING) {
                if (raw[0] > actual)
                    raw[0] = actual & ~1u;
                s->valid = 1;
            }
            pt.cap_str_slot++;
            break;
        }

        default:
            break;
    }
    return;

fail:
    if (pt.fail_code == 0)
        pt.fail_code = 10 + pt.cap_step;
    pt.host_state = HS_FAILED;
}

static void capture_task(void) {
    bool issued = false;

    if (pt.cap_busy)
        return;

    if ((uint32_t)(time_us_32() - pt.cap_started) > PT_CAPTURE_LIMIT_US) {
        pt.fail_code  = 30 + pt.cap_step;
        pt.host_state = HS_FAILED;
        return;
    }

    switch (pt.cap_step) {
        case CAP_DEVICE:
            issued = tuh_descriptor_get_device(pt.daddr, pt.dev_desc, sizeof(pt.dev_desc), capture_cb, 0);
            break;

        case CAP_CFG_HEADER:
            issued = tuh_descriptor_get_configuration(
                pt.daddr, 0, pt.cfg_desc, sizeof(tusb_desc_configuration_t), capture_cb, 0);
            break;

        case CAP_CFG_FULL: {
            uint16_t total = tu_le16toh(((tusb_desc_configuration_t const *)pt.cfg_desc)->wTotalLength);
            issued = tuh_descriptor_get_configuration(pt.daddr, 0, pt.cfg_desc, total, capture_cb, 0);
            break;
        }

        case CAP_LANGID:
            issued = tuh_descriptor_get_string(pt.daddr, 0, 0, pt.str[0].raw, PT_STR_MAX, capture_cb, 0);
            break;

        case CAP_STRINGS:
            /* Skip unused slots; when we run out, the capture is complete */
            while (pt.cap_str_slot < PT_STR_SLOTS && pt.str[pt.cap_str_slot].index == 0)
                pt.cap_str_slot++;

            if (pt.cap_str_slot >= PT_STR_SLOTS) {
                pt.cap_step = CAP_DONE;
                pt.gen++;
                pt.host_state = HS_READY;
                return;
            }

            issued = tuh_descriptor_get_string(pt.daddr,
                                               pt.str[pt.cap_str_slot].index,
                                               pt.langid,
                                               pt.str[pt.cap_str_slot].raw,
                                               PT_STR_MAX,
                                               capture_cb,
                                               0);
            break;

        default:
            return;
    }

    /* If the control pipe was busy we simply try again on the next pass */
    if (issued)
        pt.cap_busy = true;
}

/* ==================================================== *
 * Host side (core1): interrupt IN data, keyboard -> computer
 * ==================================================== */

/* Queue one packet for the computer. Returns false if polling must pause until it catches up. */
static bool relay_packet(uint8_t instance, uint8_t itf, uint8_t const *data, uint16_t len) {
    if (itf == PT_NO_ITF || !pt.clone_active || len > PT_PKT_MAX)
        return true;

    pt.rx_pkt.len = (uint8_t)len;
    memcpy(pt.rx_pkt.data, data, len);

    if (queue_try_add(&pt.in_q[itf], &pt.rx_pkt))
        COUNT_IN(in_fwd, itf);
    else
        COUNT_IN(in_drop, itf);

    /* Backpressure: stop polling this interface until the computer catches up */
    if (queue_is_full(&pt.in_q[itf])) {
        if (instance < CFG_TUH_HID)
            pt.rearm_pending[instance] = true;
        return false;
    }

    return true;
}

uint8_t pt_host_report(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
    if (!pt_is_target(dev_addr) || instance >= CFG_TUH_HID)
        return PT_RX_REARM | PT_RX_PARSE;

    uint8_t itf = pt.inst2itf[instance];
    uint8_t mps = (itf != PT_NO_ITF && pt.ep_mps[itf]) ? pt.ep_mps[itf] : PT_PKT_MAX;

    /* Long reports arrive one packet at a time, a full packet means more follows */
    bool continuation          = pt.rx_continues[instance];
    pt.rx_continues[instance] = (len >= mps);

    if (continuation) {
        if (pt.drop_rest[instance]) {
            pt.drop_rest[instance] = pt.rx_continues[instance];
            return PT_RX_REARM;
        }
    } else if (tuh_hid_interface_protocol(dev_addr, instance) == HID_ITF_PROTOCOL_KEYBOARD) {
        /* Start of a report on the actual keyboard interface. DeskHop's parser looks at it
           first (hotkeys, which output is active), then pt_host_report_finish() decides
           whether this computer gets to see it. */
        return PT_RX_REARM | PT_RX_PARSE | PT_RX_HELD;
    }

    /* Everything else (Touch ID, device management, vendor data) is relayed untouched */
    return relay_packet(instance, itf, report, len) ? PT_RX_REARM : 0;
}

/* Second half for PT_RX_HELD reports. "understood" tells us DeskHop's parser knew the report
   (keys, media keys), which makes it subject to hotkeys and to the active output. */
uint8_t pt_host_report_finish(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len, bool understood) {
    (void)dev_addr;
    uint8_t itf  = pt.inst2itf[instance];
    bool forward = true;

    if (understood && len > 0) {
        /* Remember its shape, so "everything released" can be sent when the output moves away */
        for (int i = 0; i < PT_GATED_IDS; i++) {
            if (pt.gated[i].len == 0 || pt.gated[i].id == report[0]) {
                pt.gated[i].id  = report[0];
                pt.gated[i].len = (uint8_t)len;
                break;
            }
        }

        forward = CURRENT_BOARD_IS_ACTIVE_OUTPUT && !pt.swallow;
    }

    if (!forward) {
        pt.drop_rest[instance] = pt.rx_continues[instance];
        return PT_RX_REARM;
    }

    return relay_packet(instance, itf, report, len) ? PT_RX_REARM : 0;
}

/* When the output moves to the other computer, this one would otherwise be left with
   whatever keys were down at that moment (the hotkey itself, usually). */
static void release_task(void) {
    bool active = CURRENT_BOARD_IS_ACTIVE_OUTPUT;

    if (pt.was_active && !active && pt.clone_active && pt.kbd_itf != PT_NO_ITF) {
        for (int i = 0; i < PT_GATED_IDS; i++) {
            if (pt.gated[i].len == 0)
                continue;

            pt.rx_pkt.len = pt.gated[i].len;
            memset(pt.rx_pkt.data, 0, sizeof(pt.rx_pkt.data));
            pt.rx_pkt.data[0] = pt.gated[i].id;
            queue_try_add(&pt.in_q[pt.kbd_itf], &pt.rx_pkt);
        }
    }

    pt.was_active = active;
}

static void rearm_task(void) {
    for (uint8_t i = 0; i < CFG_TUH_HID; i++) {
        if (!pt.rearm_pending[i])
            continue;

        uint8_t itf = pt.inst2itf[i];

        if (itf != PT_NO_ITF && queue_is_full(&pt.in_q[itf]) && pt.clone_active)
            continue;

        if (tuh_hid_receive_report(pt.daddr, i))
            pt.rearm_pending[i] = false;
    }
}

/* ==================================================== *
 * Host side (core1): control requests, computer -> keyboard
 * ==================================================== */

static void host_ctl_done(bool ok, uint16_t len) {
    pt.ctl_inflight    = false;
    pt.xchg.ok         = ok;
    pt.xchg.actual_len = len;
    pt.xchg.state      = X_DONE;
}

static void host_ctl_cb(tuh_xfer_t *xfer) {
    bool ok = (xfer->result == XFER_RESULT_SUCCESS);

    /* Keyboard was unplugged in the meantime, the request has been failed already */
    if (!pt.ctl_inflight)
        return;

    TRACE_HOST(ok ? 'c' : 'e', &pt.xchg.setup, ok ? (uint16_t)xfer->actual_len : (uint16_t)xfer->result);
    host_ctl_done(ok, (uint16_t)MIN(xfer->actual_len, (uint32_t)PT_CTRL_MAX));
}

/* Send the request core0 handed over to the keyboard. The transfer works directly
   on the shared buffer, which belongs to this core until it is marked X_DONE. */
static void host_ctl_task(void) {
    if (pt.ctl_inflight || pt.xchg.state != X_REQ)
        return;

    if (!pt.ctl_host_waiting) {
        pt.ctl_host_waiting = true;
        pt.ctl_host_since   = time_us_32();
    }

    tuh_xfer_t xfer = {
        .daddr       = pt.daddr,
        .ep_addr     = 0,
        .setup       = &pt.xchg.setup,
        .buffer      = pt.xchg.setup.wLength ? pt.xchg.data : NULL,
        .complete_cb = host_ctl_cb,
        .user_data   = 0,
    };

    pt.ctl_inflight = true;
    pt.xchg.state   = X_BUSY;

    if (tuh_control_xfer(&xfer)) {
        pt.ctl_host_waiting = false;
        TRACE_HOST('h', &pt.xchg.setup, 0);
        return;
    }

    /* Control pipe busy. Try again on the next pass, but not forever. */
    pt.ctl_inflight = false;
    pt.xchg.state   = X_REQ;

    if ((uint32_t)(time_us_32() - pt.ctl_host_since) > 1000000) {
        pt.ctl_host_waiting = false;
        TRACE_HOST('g', &pt.xchg.setup, 0);
        host_ctl_done(false, 0);
    }
}

void pt_host_task(device_t *state) {
    (void)state;

    switch (pt.host_state) {
        case HS_CAPTURE:
            capture_task();
            break;

        case HS_READY:
            release_task();
            rearm_task();
            host_ctl_task();
            break;

        default:
            break;
    }
}

/* ==================================================== *
 * Device side (core0): class driver for the cloned interfaces
 * ==================================================== */

static void stall_control(uint8_t rhport) {
    dcd_edpt_stall(rhport, 0x00); /* EP0 OUT */
    dcd_edpt_stall(rhport, 0x80); /* EP0 IN */
}

/* The computer moved on (new request, bus reset) while we still owed it an answer.
   Whatever the keyboard says to the old request is ignored. */
static void ctl_abandon(void) {
    if (pt.ctl_state == C_IDLE)
        return;

    TRACE_DEV('P', &pt.ctl_setup, pt.ctl_state);
    pt.ctl_seq++;
    pt.ctl_state = C_IDLE;
}

static void ptd_init(void) {
}

static bool ptd_deinit(void) {
    return true;
}

static void ptd_reset(uint8_t rhport) {
    (void)rhport;
    memset(pt.ep_in, 0, sizeof(pt.ep_in));
    ctl_abandon();
}

/* Claim every interface of the cloned configuration and open its endpoints. */
static uint16_t ptd_open(uint8_t rhport, tusb_desc_interface_t const *desc_itf, uint16_t max_len) {
    if (!pt.clone_active || desc_itf->bInterfaceNumber >= pt.num_itf)
        return 0;

    uint8_t const itf = desc_itf->bInterfaceNumber;
    uint16_t      len = desc_itf->bLength;
    uint8_t const *p  = tu_desc_next(desc_itf);

    while (len < max_len && tu_desc_len(p) >= 2 && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
        if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;

            TU_ASSERT(usbd_edpt_open(rhport, ep), 0);

            if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN)
                pt.ep_in[itf] = ep->bEndpointAddress;
        }
        len += tu_desc_len(p);
        p = tu_desc_next(p);
    }

    return len;
}

/* Relay a class or vendor request to the real keyboard.
 *
 * Nothing is answered here. The request is noted, pt_device_task hands it to core1,
 * and only once the keyboard has completed it do we send the data or status stage
 * (or a STALL, if that is what the keyboard said). Until then the controller NAKs,
 * exactly like a device that is still busy, so the computer never gets ahead of
 * the keyboard and every outcome is the keyboard's own. */
static bool relay_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    bool const is_in = (request->bmRequestType_bit.direction == TUSB_DIR_IN);

    if (stage == CONTROL_STAGE_SETUP) {
        ctl_abandon();

        if (request->wLength > PT_CTRL_MAX) {
            TRACE_DEV('L', request, 0);
            return false;
        }

        pt.ctl_setup = *request;
        pt.ctl_since = time_us_32();

        if (!is_in && request->wLength > 0) {
            /* Collect the data stage first */
            pt.ctl_state = C_RX_DATA;
            TRACE_DEV('O', request, 0);
            return tud_control_xfer(rhport, request, pt.ctl_buf, request->wLength);
        }

        pt.ctl_state = C_WAIT_SLOT;
        TRACE_DEV(is_in ? 'I' : 'N', request, 0);
        return true;
    }

    if (stage == CONTROL_STAGE_DATA && !is_in && pt.ctl_state == C_RX_DATA) {
        /* Data is in, hold back the status stage until the keyboard has taken it */
        pt.ctl_state = C_WAIT_SLOT;
        pt.ctl_since = time_us_32();
        usbd_control_defer_status();
        TRACE_DEV('D', request, 0);
        return true;
    }

    if (stage == CONTROL_STAGE_ACK)
        TRACE_DEV('A', request, 0);

    return true;
}

static bool ptd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    if (stage == CONTROL_STAGE_SETUP)
        ctl_abandon();

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS)
        return relay_control(rhport, stage, request);

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_STANDARD)
        return false;

    /* Standard requests: the HID and report descriptors are served from our copy */
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    TRACE_DEV('s', request, 0);

    if (request->bRequest != TUSB_REQ_GET_DESCRIPTOR)
        return false;

    uint8_t const itf = tu_u16_low(request->wIndex);

    if (itf >= pt.num_itf)
        return false;

    switch (tu_u16_high(request->wValue)) {
        case HID_DESC_TYPE_HID: {
            uint8_t *hid = pt.cfg_desc + pt.hid_desc_ofs[itf];
            return tud_control_xfer(rhport, request, hid, hid[0]);
        }

        case HID_DESC_TYPE_REPORT:
            return tud_control_xfer(rhport, request, pt.rpt_desc[itf], pt.rpt_len[itf]);

        default:
            return false;
    }
}

static bool ptd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    (void)rhport;
    (void)ep_addr;
    (void)result;
    (void)xferred_bytes;

    /* The next packet is picked up by pt_device_task */
    return true;
}

static usbd_class_driver_t const pt_driver = {
    .name            = "PASSTHROUGH",
    .init            = ptd_init,
    .deinit          = ptd_deinit,
    .reset           = ptd_reset,
    .open            = ptd_open,
    .control_xfer_cb = ptd_control_xfer_cb,
    .xfer_cb         = ptd_xfer_cb,
    .sof             = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &pt_driver;
}

/* Vendor requests are not tied to an interface, relay them as well. */
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    if (!pt.clone_active)
        return false;

    return relay_control(rhport, stage, request);
}

/* ==================================================== *
 * Device side (core0): tasks
 * ==================================================== */

static void flush_in_queues(void) {
    pt_pkt_t *discard = &pt.tx_pkt[0];

    for (int i = 0; i < PT_MAX_ITF; i++)
        while (queue_try_remove(&pt.in_q[i], discard))
            ;
}

static uint16_t cfg_total(uint8_t const *cfg) {
    return tu_le16toh(((tusb_desc_configuration_t const *)cfg)->wTotalLength);
}

static uint16_t endpoint_numbers(uint8_t const *p, uint8_t const *end) {
    uint16_t mask = 0;

    for (; p + 2 <= end && tu_desc_len(p) >= 2; p += tu_desc_len(p))
        if (tu_desc_type(p) == TUSB_DESC_ENDPOINT)
            mask |= (uint16_t)(1u << (p[2] & 0x0f));

    return mask;
}

/* Build what the computer gets to see: the keyboard's own configuration, followed by
   DeskHop's usual interfaces (keyboard, mice, debug port) moved out of its way.
   The keyboard's part stays byte for byte identical. */
static bool build_configuration(void) {
    extern uint8_t const desc_configuration[];

    uint16_t clone_len = cfg_total(pt.cfg_desc);
    uint16_t own_hdr   = desc_configuration[0];
    uint16_t own_len   = cfg_total(desc_configuration) - own_hdr;

    if (clone_len + own_len > sizeof(pt.cfg_out))
        return false;

    uint8_t *own = pt.cfg_out + clone_len;
    uint8_t *end = own + own_len;

    memcpy(pt.cfg_out, pt.cfg_desc, clone_len);
    memcpy(own, desc_configuration + own_hdr, own_len);

    /* Endpoint numbers of ours that clash with the keyboard's get the next free number */
    uint16_t used = endpoint_numbers(pt.cfg_desc, pt.cfg_desc + clone_len);
    uint16_t ours = endpoint_numbers(own, end);
    uint8_t remap[16] = {0};

    for (uint8_t n = 1; n < 16; n++) {
        if (!(ours & used & (1u << n)))
            continue;

        uint8_t m = 1;
        while (m < 16 && ((used | ours) & (1u << m)))
            m++;

        if (m >= 16)
            return false;

        remap[n] = m;
        ours |= (uint16_t)(1u << m);
    }

    /* Renumber our interfaces after the keyboard's. Our strings don't exist in the
       keyboard's string table, so the interfaces go unnamed. */
    for (uint8_t *p = own; p + 2 <= end && tu_desc_len(p) >= 2; p += tu_desc_len(p)) {
        switch (tu_desc_type(p)) {
            case TUSB_DESC_INTERFACE:
                p[2] += pt.num_itf;
                p[8] = 0;
                break;

            case TUSB_DESC_INTERFACE_ASSOCIATION:
                p[2] += pt.num_itf;
                p[7] = 0;
                break;

            case TUSB_DESC_CS_INTERFACE:
                /* CDC call management and union descriptors name interfaces too */
                if (p[2] == 0x01 && tu_desc_len(p) >= 5)
                    p[4] += pt.num_itf;
                if (p[2] == 0x06 && tu_desc_len(p) >= 5) {
                    p[3] += pt.num_itf;
                    p[4] += pt.num_itf;
                }
                break;

            case TUSB_DESC_ENDPOINT:
                if (remap[p[2] & 0x0f])
                    p[2] = (uint8_t)((p[2] & 0x80) | remap[p[2] & 0x0f]);
                break;

            default:
                break;
        }
    }

    tusb_desc_configuration_t *cfg = (tusb_desc_configuration_t *)pt.cfg_out;
    cfg->wTotalLength   = tu_htole16(clone_len + own_len);
    cfg->bNumInterfaces = (uint8_t)(pt.num_itf + desc_configuration[4]);

    return true;
}

/* Swap between "DeskHop Switch" and the cloned keyboard by dropping off the bus
   for a moment, so the computer enumerates us again with the other descriptors. */
static void switch_task(void) {
    bool want_clone = (pt.host_state == HS_READY) && !global_state.config_mode_active;
    uint32_t now    = time_us_32();

    /* Don't keep retrying a keyboard whose configuration we could not combine with ours */
    static uint8_t refused_gen;
    static bool    refused;

    if (refused && refused_gen == pt.gen)
        want_clone = false;

#ifdef DH_PASSTHROUGH_TRACE
    /* Give the computer a moment to finish its handshake, then start printing the trace */
    if (pt.dev_state == DS_CLONE && (uint32_t)(now - pt.switch_started) > PT_TRACE_CLONE_US)
        trace_hold = true;
#endif

    switch (pt.dev_state) {
        case DS_NORMAL:
            if (want_clone) {
                tud_disconnect();
                pt.switch_started = now;
                pt.dev_state      = DS_TO_CLONE;
            }
            break;

        case DS_TO_CLONE:
            if ((uint32_t)(now - pt.switch_started) < PT_SWITCH_DELAY_US)
                break;

            if (want_clone && !build_configuration()) {
                refused     = true;
                refused_gen = pt.gen;
                want_clone  = false;
            }

            if (!want_clone) {
                /* Keyboard vanished while we were off the bus (or it doesn't fit next
                   to our own interfaces): come back as ourselves */
                tud_connect();
                pt.dev_state = DS_NORMAL;
                break;
            }

            flush_in_queues();
            pt.active_gen     = pt.gen;
            pt.clone_active   = true;
            pt.dev_state      = DS_CLONE;
            pt.switch_started = now;
            tud_connect();
            break;

        case DS_CLONE:
            if (!want_clone || pt.gen != pt.active_gen) {
                tud_disconnect();
                pt.switch_started = now;
                pt.dev_state      = DS_TO_NORMAL;
            }
            break;

        case DS_TO_NORMAL:
            if ((uint32_t)(now - pt.switch_started) < PT_SWITCH_DELAY_US)
                break;

            pt.clone_active = false;
            ctl_abandon();
            flush_in_queues();
            pt.dev_state = DS_NORMAL;
            tud_connect();
            break;
    }
}

static void device_in_task(void) {
    for (uint8_t itf = 0; itf < PT_MAX_ITF; itf++) {
        uint8_t ep = pt.ep_in[itf];

        if (queue_is_empty(&pt.in_q[itf]))
            continue;

        /* Nobody is listening: wake the computer if it sleeps, and don't let data pile up */
        if (ep == 0 || !tud_mounted() || tud_suspended()) {
            if (tud_suspended() && itf == pt.kbd_itf)
                tud_remote_wakeup();

            queue_try_remove(&pt.in_q[itf], &pt.tx_pkt[itf]);
            continue;
        }

        if (usbd_edpt_busy(BOARD_TUD_RHPORT, ep) || !usbd_edpt_claim(BOARD_TUD_RHPORT, ep))
            continue;

        if (!queue_try_remove(&pt.in_q[itf], &pt.tx_pkt[itf])) {
            usbd_edpt_release(BOARD_TUD_RHPORT, ep);
            continue;
        }

        usbd_edpt_xfer(BOARD_TUD_RHPORT, ep, pt.tx_pkt[itf].data, pt.tx_pkt[itf].len);
    }
}

static void device_ctl_task(void) {
    bool const is_in = (pt.ctl_setup.bmRequestType_bit.direction == TUSB_DIR_IN);

    /* Hand the waiting request to core1 as soon as the slot is free */
    if (pt.ctl_state == C_WAIT_SLOT && pt.xchg.state == X_FREE) {
        pt.xchg.setup = pt.ctl_setup;
        pt.xchg.seq   = pt.ctl_seq;

        if (!is_in && pt.ctl_setup.wLength > 0)
            memcpy(pt.xchg.data, pt.ctl_buf, pt.ctl_setup.wLength);

        pt.ctl_state  = C_WAIT_HOST;
        pt.xchg.state = X_REQ;
    }

    /* The keyboard has answered (or failed to): pass that on to the computer */
    if (pt.xchg.state == X_DONE) {
        if (pt.ctl_state == C_WAIT_HOST && pt.xchg.seq == pt.ctl_seq) {
            pt.ctl_state = C_IDLE;

            if (!pt.xchg.ok) {
                TRACE_DEV('X', &pt.ctl_setup, 0);
                stall_control(BOARD_TUD_RHPORT);
            } else if (is_in) {
                uint16_t len = MIN(pt.xchg.actual_len, pt.ctl_setup.wLength);

                memcpy(pt.ctl_buf, pt.xchg.data, len);
                TRACE_DEV('R', &pt.ctl_setup, len);
                tud_control_xfer(BOARD_TUD_RHPORT, &pt.ctl_setup, pt.ctl_buf, len);
            } else {
                TRACE_DEV('S', &pt.ctl_setup, 0);
                tud_control_status(BOARD_TUD_RHPORT, &pt.ctl_setup);
            }
        } else {
            /* Answer to a request the computer has already given up on */
            TRACE_DEV('Z', &pt.xchg.setup, 0);
        }

        pt.xchg.state = X_FREE;
    }

    if ((pt.ctl_state == C_WAIT_SLOT || pt.ctl_state == C_WAIT_HOST) &&
        (uint32_t)(time_us_32() - pt.ctl_since) > PT_CTRL_TIMEOUT_US) {
        TRACE_DEV('T', &pt.ctl_setup, pt.ctl_state);
        pt.ctl_seq++;
        pt.ctl_state = C_IDLE;
        stall_control(BOARD_TUD_RHPORT);
    }
}

#ifdef DH_DEBUG
/* Once a second, report where the passthrough state machine is (core0, CDC debug port) */
static void debug_task(void) {
    static uint32_t last;
    uint16_t vid = 0, pid = 0;
    uint32_t now = time_us_32();

    if ((uint32_t)(now - last) < 1000000)
        return;
    last = now;

    tuh_vid_pid_get(pt.daddr ? pt.daddr : 1, &vid, &pid);

    dh_debug_printf("PT host=%u cap=%u fail=%u dev=%u daddr=%u id=%04x:%04x itf=%u rpt=%u/%u/%u/%u kbd=%u cfg=%u\r\n",
                    pt.host_state, pt.cap_step, pt.fail_code, pt.dev_state, pt.daddr, vid, pid, pt.num_itf,
                    pt.rpt_len[0], pt.rpt_len[1], pt.rpt_len[2], pt.rpt_len[3],
                    global_state.keyboard_connected, global_state.config_mode_active);

    /* Raw configuration descriptor, as far as it was read */
    static char hex[2 * 120 + 1];
    static const char digits[] = "0123456789abcdef";
    uint16_t total = tu_le16toh(((tusb_desc_configuration_t const *)pt.cfg_desc)->wTotalLength);
    uint16_t n     = MIN(total, (uint16_t)120);

    for (uint16_t i = 0; i < n; i++) {
        hex[2 * i]     = digits[pt.cfg_desc[i] >> 4];
        hex[2 * i + 1] = digits[pt.cfg_desc[i] & 0x0f];
    }
    hex[2 * n] = 0;

    dh_debug_printf("PT cfg(%u)=%s\r\n", total, hex);

#ifdef DH_PASSTHROUGH_TRACE
    if (!trace_hold)
        return;

    dh_debug_printf("PT in fwd=%lu/%lu/%lu drop=%lu/%lu/%lu\r\n", in_fwd[0], in_fwd[1], in_fwd[2],
                    in_drop[0], in_drop[1], in_drop[2]);

    for (uint8_t core = 0; core < 2; core++) {
        for (uint16_t i = 0; i < trace_count[core]; i++) {
            pt_trace_t *t = &trace_log[core][i];
            dh_debug_printf("TR%u %03u %lu %c bm=%02x rq=%02x v=%04x i=%04x l=%u a=%u\r\n", core, i, t->t_ms, t->ev,
                            t->bm, t->req, t->val, t->idx, t->len, t->aux);
        }
    }
    dh_debug_printf("TR end\r\n");
#endif
}
#endif

void pt_device_task(device_t *state) {
    (void)state;

#ifdef DH_DEBUG
    debug_task();
#endif

    switch_task();

    if (pt.dev_state != DS_CLONE)
        return;

    device_in_task();
    device_ctl_task();
}

#endif /* DH_PASSTHROUGH */
