#include "i2cdriver_worker.h"
#include "i2cdriver_app.h"
#include <cli/cli_vcp.h>
#include <furi_hal_i2c.h>
#include <furi_hal_usb.h>
#include <furi_hal_usb_cdc.h>
#include <stdio.h>
#include <string.h>

#define WorkerEvtStop  (1 << 0)
#define WorkerEvtCdcRx (1 << 1)

#define I2C_BUS      (&furi_hal_i2c_handle_external)
#define I2C_TIMEOUT  10 // ms — for reads/writes
#define I2C_PROBE_MS 3 // ms — for scan (112 addrs × 3ms = 336ms < 1s serial timeout)

// ── Ring buffer ───────────────────────────────────────────────────────────────
#define RX_BUF_SIZE 256
static uint8_t rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static void on_cdc_rx(void* context) {
    I2CDriverWorker* w = context;
    uint8_t temp[64];
    int32_t len = furi_hal_cdc_receive(0, temp, sizeof(temp));
    for(int32_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if(next != rx_tail) {
            rx_ring[rx_head] = temp[i];
            rx_head = next;
        }
    }
    furi_thread_flags_set(furi_thread_get_id(w->thread), WorkerEvtCdcRx);
}

// ── Ring buffer helpers ───────────────────────────────────────────────────────

static bool rb_get(uint8_t* out) {
    if(rx_head == rx_tail) return false;
    *out = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return true;
}

// Block until `len` bytes are in the ring buffer, with 200ms timeout.
static bool rb_get_exact(uint8_t* buf, size_t len) {
    uint32_t deadline = furi_get_tick() + 200;
    for(size_t i = 0; i < len; i++) {
        while(!rb_get(&buf[i])) {
            if(furi_get_tick() > deadline) return false;
            furi_delay_us(100);
        }
    }
    return true;
}

// ── CDC send helper ───────────────────────────────────────────────────────────
// furi_hal_cdc_send is synchronous but limited to 64 bytes per call.
// This wrapper chunks automatically.

static void cdc_send(const uint8_t* buf, uint16_t len) {
    while(len > 0) {
        uint16_t chunk = len > 64 ? 64 : len;
        furi_hal_cdc_send(0, (uint8_t*)buf, chunk);
        buf += chunk;
        len -= chunk;
        if(len > 0) furi_delay_ms(1); // Small delay between chunks
    }
}

static void cdc_send_byte(uint8_t b) {
    cdc_send(&b, 1);
}

// ── CCITT-16 CRC ─────────────────────────────────────────────────────────────
static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

static void crc_add(I2CDriverWorker* w, const uint8_t* data, size_t len) {
    uint16_t crc = w->crc;
    while(len--)
        crc = (crc_table[((crc >> 8) ^ *data++) & 0xff] ^ (crc << 8)) & 0xffff;
    w->crc = crc;
}

// ── STATUS response ───────────────────────────────────────────────────────────
// getstatus() does: r=ser.read(80); body=r[1:-1].decode(); body.split() -> 12 tokens
// Frame: '[' + 78 chars (space-padded) + ']' = 80 bytes exactly
static void send_status(I2CDriverWorker* w) {
    uint8_t buf[80];
    memset(buf, ' ', sizeof(buf));
    buf[0] = '[';
    buf[79] = ']';
    char inner[79];
    int n = snprintf(
        inner,
        sizeof(inner),
        "i2cdriver1 %s %lu 5.000 0 25.0 I 1 1 %lu 00 %04x",
        I2CD_SERIAL,
        (unsigned long)(furi_get_tick() / 1000),
        (unsigned long)w->speed_khz,
        (unsigned int)w->crc);
    if(n > 0 && n <= 78) memcpy(buf + 1, inner, (size_t)n);
    cdc_send(buf, 80);
}

// ── Command processor ─────────────────────────────────────────────────────────
static void process_protocol(I2CDriverWorker* w) {
    uint8_t cmd;
    I2CDriverApp* app = w->app;

    while(rb_get(&cmd)) {
        // '@' SYNC — Python sends this to flush/abort. NO response byte.
        if(cmd == '@') {
            w->crc = 0x0000;
            continue;
        }

        // Track stats for the screen
        if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
            app->bytes_rx++;
            app->cmd_count++;
            app->usb_connected = true;
            furi_mutex_release(app->mutex);
        }

        // 'e' ECHO — send the next byte back unchanged
        if(cmd == 'e') {
            uint8_t val;
            if(rb_get_exact(&val, 1)) {
                cdc_send_byte(val);
                if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                    app->bytes_rx++; // the echo value byte
                    app->bytes_tx++;
                    furi_mutex_release(app->mutex);
                }
            }
            continue;
        }

        // '?' STATUS → exactly 80 bytes
        if(cmd == '?') {
            send_status(w);
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                app->bytes_tx += 80;
                furi_mutex_release(app->mutex);
            }
            continue;
        }

        // 'x' BUS RESET → 1 byte: bit1=SDA, bit0=SCL (both idle-high = 0x03)
        if(cmd == 'x') {
            w->crc = 0x0000;
            cdc_send_byte(0x03);
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                app->bytes_tx++;
                furi_mutex_release(app->mutex);
            }
            continue;
        }

        // 'p' STOP — NO response byte
        if(cmd == 'p') {
            // STOP: SCL rises, then SDA rises
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                wave_push(app, 0, 0);
                wave_push(app, 0, 1); // SCL rises
                wave_push(app, 1, 1); // SDA rises (STOP)
                furi_mutex_release(app->mutex);
            }
            continue;
        }

        // '1' / '4' SET SPEED — no response
        if(cmd == '1') {
            w->speed_khz = 100;
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                app->speed_khz = 100;
                furi_mutex_release(app->mutex);
            }
            continue;
        }
        if(cmd == '4') {
            w->speed_khz = 400;
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                app->speed_khz = 400;
                furi_mutex_release(app->mutex);
            }
            continue;
        }

        // 'm' MONITOR — no-op; host sends '@' to abort
        if(cmd == 'm') {
            continue;
        }

        // 'd' SCAN → 112 bytes ('1'=present / '0'=absent, addrs 0x08..0x77)
        if(cmd == 'd') {
            uint8_t result[112];
            memset(result, '0', sizeof(result));
            furi_hal_i2c_acquire(I2C_BUS);
            for(uint8_t a7 = 0x08; a7 <= 0x77; a7++) {
                if(furi_hal_i2c_is_device_ready(I2C_BUS, (uint8_t)(a7 << 1), I2C_PROBE_MS))
                    result[a7 - 0x08] = '1';
            }
            furi_hal_i2c_release(I2C_BUS);
            cdc_send(result, sizeof(result));
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                app->bytes_tx += 112;
                furi_mutex_release(app->mutex);
            }
            continue;
        }

        // 's' START <addr8> → <ack byte>
        if(cmd == 's') {
            uint8_t addr8;
            if(!rb_get_exact(&addr8, 1)) continue;
            w->last_addr = addr8 & 0xFE;
            furi_hal_i2c_acquire(I2C_BUS);
            bool ready = furi_hal_i2c_is_device_ready(I2C_BUS, w->last_addr, I2C_TIMEOUT);
            furi_hal_i2c_release(I2C_BUS);
            cdc_send_byte(ready ? 1 : 0);
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                // START waveform: idle → SDA falls (while SCL high) → SCL falls
                wave_push(app, 1, 1);
                wave_push(app, 0, 1);
                wave_push(app, 0, 0);
                app->bytes_rx++;
                app->bytes_tx++;
                app->bus_started = ready;
                furi_mutex_release(app->mutex);
            }
            i2cdriver_log_push(app, "S 0x%02X %s", w->last_addr >> 1, ready ? "ACK" : "NAK");
            continue;
        }

        // 0xC0+(N-1) WRITE <N bytes> → <ack byte>
        if((cmd & 0xC0) == 0xC0) {
            uint8_t count = (cmd & 0x3F) + 1;
            uint8_t buf[64];
            if(!rb_get_exact(buf, count)) continue;
            furi_hal_i2c_acquire(I2C_BUS);
            bool ok = furi_hal_i2c_tx(I2C_BUS, w->last_addr, buf, count, I2C_TIMEOUT);
            furi_hal_i2c_release(I2C_BUS);
            crc_add(w, buf, count);
            cdc_send_byte(ok ? 1 : 0);
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                // Represent each byte as SCL pulsing with SDA data, then ACK
                for(uint8_t b = 0; b < count && b < 4; b++) {
                    for(int bit = 7; bit >= 0; bit--) {
                        wave_push(app, (buf[b] >> bit) & 1, 0);
                        wave_push(app, (buf[b] >> bit) & 1, 1);
                    }
                }
                wave_push(app, ok ? 0 : 1, 1); // ACK/NAK bit
                app->bytes_rx += count;
                app->bytes_tx++;
                furi_mutex_release(app->mutex);
            }
            // Log as hex
            char hexbuf[LOG_LINE_LEN];
            int hpos = snprintf(hexbuf, sizeof(hexbuf), "W%d:", count);
            for(uint8_t b = 0; b < count && hpos < (int)sizeof(hexbuf) - 3; b++) {
                hpos += snprintf(hexbuf + hpos, sizeof(hexbuf) - hpos, "%02X", buf[b]);
            }
            i2cdriver_log_push(app, "%s %s", hexbuf, ok ? "ACK" : "NAK");
            continue;
        }

        // 0x80+(N-1) READ → <N bytes>
        if((cmd & 0xC0) == 0x80) {
            uint8_t count = (cmd & 0x3F) + 1;
            uint8_t buf[64];
            memset(buf, 0xFF, count);
            furi_hal_i2c_acquire(I2C_BUS);
            furi_hal_i2c_rx(I2C_BUS, w->last_addr | 1, buf, count, I2C_TIMEOUT);
            furi_hal_i2c_release(I2C_BUS);
            crc_add(w, buf, count);
            cdc_send(buf, count);
            if(furi_mutex_acquire(app->mutex, 5) == FuriStatusOk) {
                for(uint8_t b = 0; b < count && b < 4; b++) {
                    for(int bit = 7; bit >= 0; bit--) {
                        wave_push(app, (buf[b] >> bit) & 1, 0);
                        wave_push(app, (buf[b] >> bit) & 1, 1);
                    }
                }
                app->bytes_tx += count;
                furi_mutex_release(app->mutex);
            }
            char hexbuf[LOG_LINE_LEN];
            int hpos = snprintf(hexbuf, sizeof(hexbuf), "R%d:", count);
            for(uint8_t b = 0; b < count && hpos < (int)sizeof(hexbuf) - 3; b++) {
                hpos += snprintf(hexbuf + hpos, sizeof(hexbuf) - hpos, "%02X", buf[b]);
            }
            i2cdriver_log_push(app, "%s", hexbuf);
            continue;
        }

        // Unknown — ignore
    }
}

// ── Worker thread ─────────────────────────────────────────────────────────────
static int32_t i2cdriver_worker_task(void* context) {
    I2CDriverWorker* w = context;

    // Reset ring buffer state
    rx_head = rx_tail = 0;

    // Take over VCP from CLI — exact pattern from usb_uart_bridge.c
    w->cli_vcp = furi_record_open(RECORD_CLI_VCP);

    furi_hal_usb_unlock();
    cli_vcp_disable(w->cli_vcp);
    furi_check(furi_hal_usb_set_config(&usb_cdc_single, NULL) == true);

    static const CdcCallbacks cdc_cb = {
        NULL, // [0] tx_ep_callback — not needed, furi_hal_cdc_send is synchronous
        on_cdc_rx, // [1] rx_ep_callback
        NULL, // [2] state_callback
        NULL, // [3] ctrl_line_callback
        NULL, // [4] config_callback
    };
    furi_hal_cdc_set_callbacks(0, (CdcCallbacks*)&cdc_cb, w);

    while(1) {
        uint32_t flags = furi_thread_flags_wait(
            WorkerEvtStop | WorkerEvtCdcRx, FuriFlagWaitAny, FuriWaitForever);
        if(flags & WorkerEvtStop) break;
        if(flags & WorkerEvtCdcRx) process_protocol(w);
    }

    // Restore CLI
    furi_hal_cdc_set_callbacks(0, NULL, NULL);
    furi_hal_usb_unlock();
    furi_check(furi_hal_usb_set_config(&usb_cdc_single, NULL) == true);
    cli_vcp_enable(w->cli_vcp);
    furi_record_close(RECORD_CLI_VCP);
    return 0;
}

// ── Public API ────────────────────────────────────────────────────────────────
I2CDriverWorker* i2cdriver_worker_alloc(I2CDriverApp* app) {
    I2CDriverWorker* w = malloc(sizeof(I2CDriverWorker));
    memset(w, 0, sizeof(I2CDriverWorker));
    w->app = app;
    w->speed_khz = 100;
    w->crc = 0x0000;
    w->thread = furi_thread_alloc_ex("I2CWorker", 2048, i2cdriver_worker_task, w);
    return w;
}

void i2cdriver_worker_start(I2CDriverWorker* w) {
    furi_thread_start(w->thread);
}

void i2cdriver_worker_stop(I2CDriverWorker* w) {
    furi_thread_flags_set(furi_thread_get_id(w->thread), WorkerEvtStop);
    furi_thread_join(w->thread);
}

void i2cdriver_worker_free(I2CDriverWorker* w) {
    i2cdriver_worker_stop(w);
    furi_thread_free(w->thread);
    free(w);
}
