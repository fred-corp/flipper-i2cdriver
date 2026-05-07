#include <furi.h>
#include <gui/gui.h>
#include "i2cdriver_app.h"
#include "i2cdriver_worker.h"

int32_t i2cdriver_app(void* p) {
    UNUSED(p);
    I2CDriverApp* app = malloc(sizeof(I2CDriverApp));
    memset(app, 0, sizeof(I2CDriverApp));
    
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->speed_khz = 100;
    
    I2CDriverWorker* worker = i2cdriver_worker_alloc(app);
    i2cdriver_worker_start(worker);

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, i2cdriver_draw, app);
    view_port_input_callback_set(app->view_port, i2cdriver_input, app);
    
    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, app->view_port, GuiLayerFullscreen);

    while(1) {
        if(furi_hal_gpio_read(&gpio_button_back) == false) break;
        furi_delay_ms(100);
    }

    gui_remove_view_port(gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);
    
    i2cdriver_worker_free(worker);
    furi_mutex_free(app->mutex);
    free(app);
    
    return 0;
}
