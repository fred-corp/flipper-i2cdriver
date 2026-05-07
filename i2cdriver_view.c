#include "i2cdriver_app.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// ── Log ───────────────────────────────────────────────────────────────────────

void i2cdriver_log_push(I2CDriverApp* app, const char* fmt, ...) {
    char buf[LOG_LINE_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    strncpy(app->log.lines[app->log.head], buf, LOG_LINE_LEN - 1);
    app->log.lines[app->log.head][LOG_LINE_LEN - 1] = '\0';
    app->log.head = (app->log.head + 1) % LOG_LINES;
    if(app->log.count < LOG_LINES) app->log.count++;
    furi_mutex_release(app->mutex);
}

// ── Draw ──────────────────────────────────────────────────────────────────────

void i2cdriver_draw(Canvas* canvas, void* ctx) {
    I2CDriverApp* app = ctx;
    if(furi_mutex_acquire(app->mutex, 25) != FuriStatusOk) return;

    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    if(app->view == AppViewStatus) {
        // ── Title bar ──────────────────────────────────────────────────────
        canvas_draw_rbox(canvas, 0, 0, 128, 13, 2);
        canvas_set_color(canvas, ColorWhite);
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 2, AlignCenter, AlignTop, "I2CDriver Bridge");
        canvas_set_color(canvas, ColorBlack);

        // ── USB status ─────────────────────────────────────────────────────
        canvas_set_font(canvas, FontSecondary);
        if(app->usb_connected) {
            canvas_draw_str(canvas, 2, 24, "USB: Connected");
        } else {
            canvas_draw_str(canvas, 2, 24, "USB: Waiting...");
        }

        // ── I2C speed ──────────────────────────────────────────────────────
        char spd[20];
        snprintf(spd, sizeof(spd), "I2C: %lu kHz", (unsigned long)app->speed_khz);
        canvas_draw_str(canvas, 2, 34, spd);

        // ── Stats ──────────────────────────────────────────────────────────
        char stat[28];
        snprintf(stat, sizeof(stat), "RX:%lu TX:%lu",
                 (unsigned long)app->bytes_rx,
                 (unsigned long)app->bytes_tx);
        canvas_draw_str(canvas, 2, 44, stat);

        snprintf(stat, sizeof(stat), "Cmds: %lu", (unsigned long)app->cmd_count);
        canvas_draw_str(canvas, 2, 54, stat);

        // ── Hint ───────────────────────────────────────────────────────────
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 126, 54, AlignRight, AlignTop, ">");

    } else {
        // ── Log view ───────────────────────────────────────────────────────
        canvas_draw_rbox(canvas, 0, 0, 128, 13, 2);
        canvas_set_color(canvas, ColorWhite);
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 2, AlignCenter, AlignTop, "Transaction Log");
        canvas_set_color(canvas, ColorBlack);

        canvas_set_font(canvas, FontSecondary);

        // Print up to LOG_LINES lines, oldest first
        uint8_t count = app->log.count;
        for(uint8_t i = 0; i < count && i < LOG_LINES; i++) {
            // oldest entry index
            uint8_t idx = (app->log.head + LOG_LINES - count + i) % LOG_LINES;
            canvas_draw_str(canvas, 2, 15 + i * 9, app->log.lines[idx]);
        }

        canvas_draw_str_aligned(canvas, 2, 54, AlignLeft, AlignTop, "<");
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
    // Back key is handled in main app loop to exit
}
