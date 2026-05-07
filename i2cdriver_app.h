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

// ADDED: Give the struct a name here
typedef struct I2CDriverApp {
    FuriMutex* mutex;
    FuriThread* worker_thread;
    volatile bool running;

    ViewPort* view_port;
    AppView      view;

    uint32_t     bytes_rx;
    uint32_t     bytes_tx;
    uint32_t     cmd_count;

    uint32_t     speed_khz;
    bool         bus_started;
    uint8_t      last_addr8;

    bool         usb_connected;
    TxLog        log;
} I2CDriverApp;

void    i2cdriver_draw(Canvas* canvas, void* ctx);
void    i2cdriver_input(InputEvent* event, void* ctx);
void    i2cdriver_log_push(I2CDriverApp* app, const char* fmt, ...);
