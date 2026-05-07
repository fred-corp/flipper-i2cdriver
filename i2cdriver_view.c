#include "i2cdriver_app.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// ── Log (transaction history) ─────────────────────────────────────────────────

void i2cdriver_log_push(I2CDriverApp* app, const char* fmt, ...) {
    char buf[LOG_LINE_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    // called from worker — must acquire mutex
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    strncpy(app->log.lines[app->log.head], buf, LOG_LINE_LEN - 1);
    app->log.lines[app->log.head][LOG_LINE_LEN - 1] = '\0';
    app->log.head = (app->log.head + 1) % LOG_LINES;
    if(app->log.count < LOG_LINES) app->log.count++;
    furi_mutex_release(app->mutex);
}

// ── Waveform helpers ──────────────────────────────────────────────────────────
// We store the last N SDA/SCL samples in the app struct as a bitmask ring.
// Each I2C byte transaction pushes 9 "bits" (8 data + 1 ACK).
// For the display we just show the most recent WAVE_LEN samples.

#define WAVE_W   116  // waveform width in pixels (leave 12px for "SDA"/"SCL" labels)
#define WAVE_X     12 // x start of waveform
#define SDA_Y_HI   18 // y when SDA=1 (high)
#define SDA_Y_LO   23 // y when SDA=0 (low)
#define SCL_Y_HI   30 // y when SCL=1
#define SCL_Y_LO   35 // y when SCL=0

// Draw one waveform row from a bitmask (MSB = leftmost sample).
// `nbits` = number of valid bits in `bits` (right-justified).
static void draw_wave(Canvas* canvas, uint32_t bits, uint8_t nbits,
                       uint8_t y_hi, uint8_t y_lo) {
    if(nbits == 0) {
        // No data yet — draw idle high line
        canvas_draw_line(canvas, WAVE_X, y_hi, WAVE_X + WAVE_W - 1, y_hi);
        return;
    }
    uint8_t px_per_bit = WAVE_W / nbits;
    if(px_per_bit < 1) px_per_bit = 1;

    uint8_t prev = (bits >> (nbits - 1)) & 1;
    uint8_t prev_y = prev ? y_hi : y_lo;

    for(uint8_t i = 0; i < nbits; i++) {
        uint8_t bit = (bits >> (nbits - 1 - i)) & 1;
        uint8_t y   = bit ? y_hi : y_lo;
        uint8_t x0  = WAVE_X + i * px_per_bit;
        uint8_t x1  = WAVE_X + (i + 1) * px_per_bit - 1;
        if(x1 >= WAVE_X + WAVE_W) x1 = WAVE_X + WAVE_W - 1;

        // Transition edge
        if(y != prev_y) {
            canvas_draw_line(canvas, x0, prev_y, x0, y);
        }
        // Horizontal segment
        canvas_draw_line(canvas, x0, y, x1, y);
        prev_y = y;
        prev   = bit;
    }
}

// ── Main draw callback ────────────────────────────────────────────────────────

void i2cdriver_draw(Canvas* canvas, void* ctx) {
    I2CDriverApp* app = ctx;
    if(furi_mutex_acquire(app->mutex, 50) != FuriStatusOk) return;

    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    if(app->view == AppViewStatus) {
        // ── Title bar ──────────────────────────────────────────────────────
        canvas_draw_rbox(canvas, 0, 0, 128, 12, 2);
        canvas_set_color(canvas, ColorWhite);
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 1, AlignCenter, AlignTop,
            app->usb_connected ? "I2CDriver  [USB OK]" : "I2CDriver  [wait]");
        canvas_set_color(canvas, ColorBlack);

        // ── SDA / SCL labels ───────────────────────────────────────────────
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 0, SDA_Y_LO, "SDA");
        canvas_draw_str(canvas, 0, SCL_Y_LO, "SCL");

        // ── Waveforms ──────────────────────────────────────────────────────
        uint8_t nbits = app->wave_count < WAVE_BITS ? app->wave_count : WAVE_BITS;
        // Extract the most recent nbits from the ring (stored MSB-first in wave_sda/scl)
        uint32_t sda_bits = app->wave_sda & ((1u << nbits) - 1);
        uint32_t scl_bits = app->wave_scl & ((1u << nbits) - 1);
        // Shift so MSB = oldest visible sample
        if(nbits > 0) {
            sda_bits = app->wave_sda >> (WAVE_BITS - nbits);
            scl_bits = app->wave_scl >> (WAVE_BITS - nbits);
        }
        draw_wave(canvas, sda_bits, nbits, SDA_Y_HI, SDA_Y_LO);
        draw_wave(canvas, scl_bits, nbits, SCL_Y_HI, SCL_Y_LO);

        // ── Divider ────────────────────────────────────────────────────────
        canvas_draw_line(canvas, 0, 38, 127, 38);

        // ── Hex log (last 3 transaction lines) ────────────────────────────
        canvas_set_font(canvas, FontSecondary);
        uint8_t count = app->log.count > 3 ? 3 : app->log.count;
        for(uint8_t i = 0; i < count; i++) {
            uint8_t idx = (app->log.head + LOG_LINES - count + i) % LOG_LINES;
            canvas_draw_str(canvas, 0, 40 + i * 9, app->log.lines[idx]);
        }

        // ── Bottom hint ────────────────────────────────────────────────────
        canvas_draw_str_aligned(canvas, 127, 56, AlignRight, AlignBottom, ">");

    } else {
        // ── Full transaction log view ──────────────────────────────────────
        canvas_draw_rbox(canvas, 0, 0, 128, 12, 2);
        canvas_set_color(canvas, ColorWhite);
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 1, AlignCenter, AlignTop, "I2C Log");
        canvas_set_color(canvas, ColorBlack);

        canvas_set_font(canvas, FontSecondary);
        uint8_t count = app->log.count;
        for(uint8_t i = 0; i < count && i < LOG_LINES; i++) {
            uint8_t idx = (app->log.head + LOG_LINES - count + i) % LOG_LINES;
            canvas_draw_str(canvas, 0, 14 + i * 9, app->log.lines[idx]);
        }
        canvas_draw_str_aligned(canvas, 0, 63, AlignLeft, AlignBottom, "<");
    }

    furi_mutex_release(app->mutex);
}

// ── Input ─────────────────────────────────────────────────────────────────────

void i2cdriver_input(InputEvent* event, void* ctx) {
    I2CDriverApp* app = ctx;
    if(event->type != InputTypeRelease) return;
    if(event->key == InputKeyRight) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->view = AppViewLog;
        furi_mutex_release(app->mutex);
    } else if(event->key == InputKeyLeft) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->view = AppViewStatus;
        furi_mutex_release(app->mutex);
    }
}
