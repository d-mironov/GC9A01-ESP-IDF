#include <cstdint>

#include "driver/spi_master.h"
#include "esp_system.h"

typedef uint32_t u32;
typedef uint8_t u8;

/**
 * Error codes for the GC9A01 display driver
 */
typedef enum gc9a01_err_t {
    GC9A01_OK,
    GC9A01_SPI_TX_ERROR,
} gc9a01_err_t;

typedef struct color_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

typedef struct gc9a01_cmd_t {
    // Command
    u8 cmd;
    // Data
    u8 data[16];
    // Data size (bytes)
    u8 datasize;
} gc9a01_cmd_t;


class GC9A01 {
public:
    GC9A01();
    GC9A01(spi_device_handle_t spi_);
    ~GC9A01() = default;

    // NOTE: Maybe arguments needed
    // TODO: Add arguments for pins
    gc9a01_err_t init(gpio_num_t mosi, gpio_num_t clk, gpio_num_t cs, gpio_num_t dc) const;

    gc9a01_err_t draw_pixel     (u32 x, u32 y, color_t color) const;
    gc9a01_err_t draw_string    (u32 x, u32 y, const char* str) const;
    gc9a01_err_t draw_box       (u32 x, u32 y, u32 w, u32 h, u32 thickness, color_t color) const;
    gc9a01_err_t draw_circle    (u32 x, u32 y, u32 r, u32 thickness, color_t color) const;
    gc9a01_err_t draw_line      (u32 x, u32 y, u32 r, u32 thickness, color_t color) const;

    gc9a01_err_t draw_bitmap    (u32 x, u32 y, u32 w, u32 h, u8* buffer);

    gc9a01_err_t fill_box       (u32 x, u32 y, u32 w, u32 h, color_t color) const;
    gc9a01_err_t fill_circle    (u32 x, u32 y, u32 r, color_t color) const;

    // NOTE: Arguments needed for font
    gc9a01_err_t set_font       (void);

    // Reset
    gc9a01_err_t soft_reset() const;
    gc9a01_err_t hard_reset() const;

private:
    gc9a01_err_t cmd(const u8 cmnd) const;
    gc9a01_err_t data(const u8* data, const u8 datasize) const;
    spi_device_handle_t spi_;
    gpio_num_t mosi_;
    gpio_num_t clk_;
    gpio_num_t cs_;
    gpio_num_t dc_;
};
