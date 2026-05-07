#pragma once

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>
#include <furi_hal_usb_cdc.h>

#define I2C_BUS         (&furi_hal_i2c_handle_external)
#define I2C_TIMEOUT_MS  10

#define CMD_ECHO        'e'
#define CMD_STATUS      '?'
#define CMD_SYNC        '@'

#define I2CD_SERIAL     "FLIP0001"

typedef enum {
    AppViewStatus = 0,
    AppViewLog,
} AppView;

#define LOG_LINES    6
#define LOG_LINE_LEN 22

typedef struct {
    char    lines[LOG_LINES][LOG_LINE_LEN];
    uint8_t head;
    uint8_t count;
} TxLog;

// Waveform ring: we store the last WAVE_BITS SDA and SCL samples as a bitmask.
// Bit 31 = oldest sample, bit 0 = newest. We shift left on each new sample.
#define WAVE_BITS 32

typedef struct I2CDriverApp {
    FuriMutex*    mutex;
    volatile bool running;

    ViewPort*  view_port;
    AppView    view;

    // Stats
    uint32_t   bytes_rx;
    uint32_t   bytes_tx;
    uint32_t   cmd_count;
    uint32_t   speed_khz;
    bool       bus_started;
    uint8_t    last_addr8;
    bool       usb_connected;

    // Waveform (SDA/SCL bitmask ring, updated by worker under mutex)
    uint32_t   wave_sda;    // MSB = oldest sample
    uint32_t   wave_scl;
    uint8_t    wave_count;  // total samples pushed (saturates at WAVE_BITS)

    // Log
    TxLog      log;
} I2CDriverApp;

void i2cdriver_draw(Canvas* canvas, void* ctx);
void i2cdriver_input(InputEvent* event, void* ctx);
void i2cdriver_log_push(I2CDriverApp* app, const char* fmt, ...);

// Push one SDA/SCL sample into the waveform ring (call under mutex).
static inline void wave_push(I2CDriverApp* app, uint8_t sda, uint8_t scl) {
    app->wave_sda = (app->wave_sda << 1) | (sda & 1);
    app->wave_scl = (app->wave_scl << 1) | (scl & 1);
    if(app->wave_count < WAVE_BITS) app->wave_count++;
}
