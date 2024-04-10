#include <cstdint>

#include "driver/spi_master.h"
#include "esp_system.h"

typedef uint32_t u32;
typedef uint8_t u8;


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

    /**
     * Error codes for the GC9A01 display driver
     */
    enum Error{
        OK,
        SPI_TX_ERROR,
    };

    // NOTE: Maybe arguments needeODO: Add arguments for pin
    Error init(gpio_num_t mosi, gpio_num_t clk, gpio_num_t cs, gpio_num_t dc) const;
    Error draw_pixel     (u32 x, u32 y, color_t color) const;
    Error draw_string    (u32 x, u32 y, const char* str) const;
    Error draw_box       (u32 x, u32 y, u32 w, u32 h, u32 thickness, color_t color) const;
    Error draw_circle    (u32 x, u32 y, u32 r, u32 thickness, color_t color) const;
    Error draw_line      (u32 x, u32 y, u32 r, u32 thickness, color_t color) const;
    Error draw_bitmap    (u32 x, u32 y, u32 w, u32 h, u8* buffer) const;

    Error fill_box       (u32 x, u32 y, u32 w, u32 h, color_t color) const;
    Error fill_circle    (u32 x, u32 y, u32 r, color_t color) const;

    // NOTE: Arguments needed for font
    Error set_font       (void);

    // Reset
    Error soft_reset() const;
    Error hard_reset() const;

private:
    Error cmd(const u8 cmnd) const;
    Error data(const u8* data, const u8 datasize) const;
    spi_device_handle_t spi_;
    gpio_num_t mosi_;
    gpio_num_t clk_;
    gpio_num_t cs_;
    gpio_num_t dc_;
};