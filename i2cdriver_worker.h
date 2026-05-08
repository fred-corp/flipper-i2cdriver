#pragma once
#include <furi.h>

// Forward declaration of the struct name only
struct I2CDriverApp;

typedef struct {
    FuriThread* thread;
    struct I2CDriverApp* app;
    bool is_running;
    uint8_t last_addr;
    uint32_t speed_khz;
    uint16_t crc;
    void* cli_vcp;
} I2CDriverWorker;

I2CDriverWorker* i2cdriver_worker_alloc(struct I2CDriverApp* app);
void i2cdriver_worker_start(I2CDriverWorker* instance);
void i2cdriver_worker_stop(I2CDriverWorker* instance);
void i2cdriver_worker_free(I2CDriverWorker* instance);
