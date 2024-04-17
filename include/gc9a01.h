#include <cstdint>

#include "driver/spi_master.h"
#include "esp_system.h"

#define COLOR_MODE_MCU_12BIT 0x03
#define COLOR_MODE_MCU_16BIT 0x05
#define COLOR_MODE_MCU_18BIT 0x06

#define GC9A01_WIDTH        240
#define GC9A01_HEIGHT       240
#define GC9A01_PIXELS       57600

#define REDSHIFT    11
#define GREENSHIFT  5

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;


typedef struct gc9a01_cmd_t {
    // Command
    u8 cmd;
    // Data
    u8 data[16];
    // Data size (bytes)
    u8 datasize;
} gc9a01_cmd_t;


// TODO: Maybe move back to `esp_err_t`
// TODO: Add buffer mode 
class GC9A01 {
public:
    GC9A01();
    GC9A01(spi_device_handle_t spi_, gpio_num_t mosi, gpio_num_t clk, gpio_num_t cs, gpio_num_t dc, gpio_num_t rst);
    ~GC9A01() = default;

    /**
     * Error codes for the GC9A01 display driver
     */
    enum Error{
        OK,
        SPI_TRANSMIT_ERROR,
        INVALID_ARGUMENT
    };

    struct Color {
        u8 r;
        u8 g;
        u8 b;

        u16 to_rgb444() const {
            // Scale red value to 4 bits
            u16 r = this->r * (15.0 / 255.0);
            // Scale green value to 4 bits
            u16 g = this->g * (15.0 / 255.0);
            // Scale blue value to 4 bits
            u16 b = this->b * (15.0 / 255.0);    
            return (r << 8) | (g << 4) | b;
        }

        u16 to_12bit() const {
            return to_rgb444();
        }

        u16 to_rgb565() const {
            // Scale red value to 5 bits
            u16 r = this->r * (31.0 / 255.0);
            // Scale green value to 6 bits
            u16 g = this->g * (63.0 / 255.0);
            // Scale blue value to 5 bits
            u16 b = this->b * (31.0 / 255.0);    
            return (r << REDSHIFT) | (g << GREENSHIFT) | b;
        }
        u16 to_16bit() const {
            return to_rgb565();
        }

        u32 to_rgb666() const {
            // Scale red value to 6 bits
            u32 r = this->r * (63.0 / 255.0);
            // Scale green value to 6 bits
            u32 g = this->g * (63.0 / 255.0);
            // Scale blue value to 6 bits
            u32 b = this->b * (63.0 / 255.0);    
            return (r << 12) | (g << 6) | b;
        }

        u32 to_18bit() const {
            return to_rgb666();
        }
    };

    // NOTE: Maybe arguments needeODO: Add arguments for pin
    Error init          () const;
    Error display_on    () const;
    Error display_off   () const;
    Error invert        (const bool invert) const;
    Error clear         () const;
    
    Error set_pixel     (u16 x, u16 y, Color color) const;
    Error fill_rect     (Color color) const;
    Error fill          (Color color) const;

    // Reset
    Error soft_reset() const;
    Error hard_reset() const;

private:
    Error cmd(const u8 cmnd) const;
    Error send_byte(const u8 byte) const;
    Error send_word(const u16 word) const;
    Error data(const u8* data, const u32 datasize) const;
    Error set_write_window(const u8 x, const u8 y, const u8 w, const u8 h) const;
    spi_device_handle_t spi_;
    gpio_num_t mosi_;
    gpio_num_t clk_;
    gpio_num_t cs_;
    gpio_num_t dc_;
    gpio_num_t rst_;
};
