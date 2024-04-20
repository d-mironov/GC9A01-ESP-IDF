/*
 * @author Daniel Mironov
 * @copyright Copyright (c) 2024, Daniel Mironov
 * @license MIT
 * @file gc9a01.cpp
 * @brief GC9A01 display driver
 * 
 * 
 */

#include <cstring>
#include <stdio.h>
#include <algorithm>
#include <array>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "gc9a01.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"


#define CMD_SWRESET 0x01
#define CMD_ID 0x04
#define CMD_DISPLAY_STATUS 0x09
#define CMD_SLEEP_ON 0x10
#define CMD_SLEEP_OFF 0x11
#define CMD_PARTIAL_MODE_ON 0x12
#define CMD_NORMAL_MODE_ON 0x13
#define CMD_INVERT_OFF 0x20
#define CMD_INVERT_ON 0x21
#define CMD_DISPLAY_OFF 0x28
#define CMD_DISPLAY_ON 0x29
#define CMD_COLADDRSET 0x2A
#define CMD_ROWADDRSET 0x2B
#define CMD_MEMORY_WRITE 0x2C
#define CMD_PARTIAL_AREA 0x30
#define CMD_VERTICAL_SCROLL 0x33
#define CMD_TEARING_OFF 0x34
#define CMD_TEARING_ON 0x35
#define CMD_MEM_ACCESS_CTL 0x36
#define CMD_VERTICAL_SCROLL_START_ADDR 0x37
#define CMD_IDLE_OFF 0x38
#define CMD_IDLE_ON 0x39
#define CMD_COLMOD 0x3A
#define CMD_WRITE_MEM_CONTINUE 0x3C
#define CMD_SET_TEAR_SCANLINE 0x44
#define CMD_GET_SCANLINE 0x45
#define CMD_BRIGHTNESS 0x51
#define CMD_CTRL_DISPLAY 0x53
#define CMD_READ_ID1 0xDA
#define CMD_READ_ID2 0xDB
#define CMD_READ_ID3 0xDC

// Extended command set as found in the datasheet
#define CMD_RGB_INTERFACE_SIGNAL_CTRL 0xB0
#define CMD_BLANKING_PORCH_CTRL 0xB5
#define CMD_DISPLAY_FUNCTION_CTRL 0xB6
#define CMD_TE_CTRL 0xBA
#define CMD_INTERFACE_CTRL 0xF6
#define CMD_POWER_CRITERION_CTRL 0xC1
#define CMD_VCORE_VOLTAGE_CTRL 0xA7
#define CMD_VREG1A_VOLTAGE_CTRL 0xC3
#define CMD_VREG1B_VOLTAGE_CTRL 0xC4
#define CMD_VREG2A_VOLTAGE_CTRL 0xC9
#define CMD_FRAMERATE 0xE8
#define CMD_SPI_2_DATA_CTRL 0xE9
#define CMD_CHARGE_PUMP_FREQENT_CTRL 0xEC
#define CMD_INTER_REG_EN_1 0xFE
#define CMD_INTER_REG_EN_2 0xEF
#define CMD_SET_GAMMA_1 0xF0
#define CMD_SET_GAMMA_2 0xF1
#define CMD_SET_GAMMA_3 0xF2
#define CMD_SET_GAMMA_4 0xF3


#define MADCTL_MY   0x80
#define MADCTL_MX   0x40
#define MADCTL_MV   0x20
#define MADCTL_BGR  0x08

#define ERROR_CHECK(error)\
if ((error) != OK) {      \
    return error;         \
}

#define NUM_INIT_COMMANDS 46

#define GC9A01_RST_DELAY 200

// TODO: Find out if ESP_LOGD is optimized out when log level is lower
#define LOG(msg, args...) ESP_LOGD("gc9a01", msg, ##args)

/*
 * Commands to initialize the screen
 * Many commands are unknown since these
 * commands come from the manufacturer and
 * were not explained and as far as I know
 * are not in the datasheet
 */
static const gc9a01_cmd_t gc9a01_init_cmds[] {
    {CMD_INTER_REG_EN_2, {0}, 0},                                                           // TODO
    {0xeb, {0x14}, 1},                                                                      // Unknown command
    {CMD_INTER_REG_EN_1, {0}, 0},                                                           // TODO
    {CMD_INTER_REG_EN_2, {0}, 0},                                                           // TODO
    {0xeb, {0x14}, 1},                                                                      // Unknown command
    {0x84, {0x40}, 1},                                                                      // Unknown command
    {0x85, {0xff}, 1},                                                                      // Unknown command
    {0x86, {0xff}, 1},                                                                      // Unknown command
    {0x87, {0xff}, 1},                                                                      // Unknown command
    {0x88, {0x0a}, 1},                                                                      // Unknown command
    {0x89, {0x21}, 1},                                                                      // Unknown command
    {0x8a, {0x00}, 1},                                                                      // Unknown command
    {0x8b, {0x80}, 1},                                                                      // Unknown command
    {0x8c, {0x01}, 1},                                                                      // Unknown command
    {0x8d, {0x01}, 1},                                                                      // Unknown command
    {0x8e, {0xff}, 1},                                                                      // Unknown command
    {0x8f, {0xff}, 1},                                                                      // Unknown command
    {CMD_DISPLAY_FUNCTION_CTRL, {0x00, 0x20}, 2},                                           // TODO
    {CMD_MEM_ACCESS_CTL, {0x08}, 1},                                                        // TODO
    {CMD_COLMOD, {COLOR_MODE_MCU_16BIT}, 1},                                                // TODO
    {0x90, {0x08, 0x08, 0x08, 0x08}, 4},                                                    // Unknown command
    {0xbd, {0x06}, 1},                                                                      // Unknown command
    {0xbc, {0x00}, 1},                                                                      // Unknown command
    {0xff, {0x60, 0x01, 0x04}, 3},                                                          // Unknown command
    {CMD_VREG1A_VOLTAGE_CTRL, {0x13}, 1},                                                   // TODO
    {CMD_VREG1B_VOLTAGE_CTRL, {0x13}, 1},                                                   // TODO
    {CMD_VREG2A_VOLTAGE_CTRL, {0x22}, 1},                                                   // TODO
    {0xbe, {0x11}, 1},                                                                      // Unknown command
    {0xe1, {0x10, 0x0e}, 2},                                                                // Unknown command
    {0xdf, {0x21, 0x0c, 0x02}, 3},                                                          // Unknown command
    {CMD_SET_GAMMA_1, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6},                             // TODO
    {CMD_SET_GAMMA_2, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6},                             // TODO
    {CMD_SET_GAMMA_3, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6},                             // TODO
    {CMD_SET_GAMMA_4, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6},                             // TODO
    {0xed, {0x1b, 0x0b}, 2},                                                                // Unknown command
    {0xae, {0x77}, 1},                                                                      // Unknown command
    {0xcd, {0x63}, 1},                                                                      // Unknown command
    // Apparently the next line causes issues for some people (TODO: need to look into)
    // {0x70, {0x07,0x07,0x04,0x0e,0x0f,0x09,0x07,0x08,0x03}, 9},
    {CMD_FRAMERATE, {0x34}, 1},                                                             // TODO
    {0x62, {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70}, 12},   // Unknown command
    {0x63, {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70}, 12},   // Unknown command
    {0x64, {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}, 7},                                  // Unknown command
    {0x66, {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}, 10},               // Unknown command
    {0x67, {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}, 10},               // Unknown command
    {0x74, {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00}, 7},                                  // Unknown command
    {0x98, {0x3e, 0x07}, 2},                                                                // Unknown command
    {CMD_TEARING_ON, {0}, 0},                                                               // TODO
    // TODO: Check if commands below are needed
    {CMD_INVERT_ON, {0x00}, 0},
    {CMD_SLEEP_OFF, {0x80}, 1},
    {CMD_DISPLAY_ON, {0x80}, 1},
    // {0, {0}, 0xff}, // END
};


void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(static_cast<gpio_num_t>(CONFIG_GC9A01_PIN_NUM_DC), dc);
}

GC9A01::GC9A01() :
    host_(static_cast<spi_host_device_t>(CONFIG_GC9A01_SPI_HOST)),
    mosi_(static_cast<gpio_num_t>(CONFIG_GC9A01_PIN_NUM_MOSI)),
    clk_(static_cast<gpio_num_t>(CONFIG_GC9A01_PIN_NUM_SCK)),
    cs_(static_cast<gpio_num_t>(CONFIG_GC9A01_PIN_NUM_CS)),
    dc_(static_cast<gpio_num_t>(CONFIG_GC9A01_PIN_NUM_DC)),
#ifdef CONFIG_GC9A01_RESET_USED
    rst_(static_cast<gpio_num_t>(CONFIG_GC9A01_PIN_NUM_RST))
#else
    rst_(GPIO_NUM_NC)
#endif
{
}

GC9A01::GC9A01(gpio_num_t mosi, gpio_num_t clk, gpio_num_t cs, gpio_num_t dc, gpio_num_t rst) : 
    mosi_(mosi), clk_(clk), cs_(cs), dc_(dc), rst_(rst)
{

}


/**
 * @brief Send `cmnd` to the display
 *
 * @param cmnd Command to send
 * @return `OK` if the command was sent successfully, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::cmd(const u8 cmnd) const {
    esp_err_t err;
    spi_transaction_t t;

    // Zero out the transmission ??
    std::memset(&t, 0, sizeof(t));
    // Command is 8 bits
    t.length = 8;
    // Data is the command itself
    t.tx_buffer = &cmnd;
    // D/C needs to be set to 0
    t.user = (void *)0;

    LOG("CMD: 0x%02x", cmnd);

    // Transmit data
    err = spi_device_polling_transmit(this->spi_, &t);
    // assert(err == ESP_OK);
    return err == ESP_OK ? OK : SPI_TRANSMIT_ERROR;
}

/**
 * @brief Send `data` to the display
 * 
 * @param data Data to send
 * @param datasize Size of the data
 * @return `OK` if the data was sent successfully, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::data(const u8* data, const u32 datasize) const { 
    esp_err_t err;
    spi_transaction_t t;

    // LOG("DATA: %ld bytes", datasize);

    // no data
    if (datasize == 0 or data == nullptr) {
        return OK;
    }
    // Zero out the transmission ??
    std::memset(&t, 0, sizeof(t));
    t.length = 8 * datasize;
    t.tx_buffer = data;
    t.user = (void *)1; // TODO: When 1 and when 0?
    err = spi_device_polling_transmit(this->spi_, &t);
    return err == ESP_OK ? OK : SPI_TRANSMIT_ERROR;
}

/**
 * Perform a hard reset of the display.
 *
 * @returns `OK`
 */
GC9A01::Error GC9A01::hard_reset() const {
    LOG("Hard reset");
    gpio_set_level(this->rst_, 1);
    vTaskDelay(GC9A01_RST_DELAY / portTICK_PERIOD_MS);
    gpio_set_level(this->rst_, 0);
    vTaskDelay(GC9A01_RST_DELAY / portTICK_PERIOD_MS);
    gpio_set_level(this->rst_, 1);
    vTaskDelay(GC9A01_RST_DELAY / portTICK_PERIOD_MS);
    return OK;
}

/**
 * @brief Soft reset the display
 * @return `OK` if the reset was successful, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::soft_reset() const {
    LOG("Soft reset");
    return cmd(CMD_SWRESET);
}

/**
 * @brief Initialize the display with the configured settings in `menuconfig`
 * 
 * This function initializes the display with the settings configured in `menuconfig`.
 * It sets up the GPIO pins, SPI bus and device, and sends the initialization commands.
 * 
 * @return `OK` if the initialization was successful, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::init() 
{
    LOG("Display Initialization");
    LOG("SPI Host: %d", host_);
    esp_err_t esp_err;
    // GPIO setup
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << this->dc_),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    #ifdef CONFIG_GC9A01_RESET_USED
    io_conf.pin_bit_mask |= (1ULL << rst_);
    #endif
    esp_err = gpio_config(&io_conf);
    ESP_ERROR_CHECK(esp_err);

    // SPI setup
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi_,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = clk_,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CONFIG_GC9A01_HEIGHT * CONFIG_GC9A01_WIDTH * 2
        
    };
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 80000000, // 40MHz
        .spics_io_num = cs_,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 7,
        .pre_cb = lcd_spi_pre_transfer_callback,
    };

    esp_err = spi_bus_initialize(this->host_, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(esp_err);
    esp_err = spi_bus_add_device(this->host_, &devcfg, &this->spi_);
    ESP_ERROR_CHECK(esp_err);


    hard_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    soft_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    Error result = OK;
    for (const auto& init_cmd : gc9a01_init_cmds)
    {
        if ((result = cmd(init_cmd.cmd)) != OK)
        {
            break;
        }
        if ((result = data(init_cmd.data, init_cmd.datasize)) != OK)
        {
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    clear();
    return result;
}

/**
 * @brief Turns the display off
 * 
 * @return `OK` on success, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::display_off() const {
    return cmd(CMD_DISPLAY_OFF);
}

/**
 * @brief Turns the display on
 * 
 * @return `OK` on success, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::display_on() const {
    return cmd(CMD_DISPLAY_ON);
}

/**
 * @brief Sets the display color inversion
 * @param inv `true` to invert the display, `false` to revert the inversion
 * @return `OK` on success, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::invert(const bool inv) const {
    return cmd(inv ? CMD_INVERT_ON : CMD_INVERT_OFF);
}

/**
 * @brief Set the rotation and mirroring of the display
 * @param rotation 
 * @return `OK` on success, `INVALID_ARGUMENT` if the rotation is invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::set_rotation(const u8 rotation) const {
    Error err;
    u8 madctl = 0;
    // TODO: may need to optimize the modulus
    switch (rotation % 8) {
        case 0:
            madctl = 0x00;
            break;
        case 1:
            madctl = MADCTL_MY;
            break;
        case 2:
            madctl = MADCTL_MX;
            break;
        case 3:
            madctl = MADCTL_MX | MADCTL_MY;
            break;
        case 4:
            madctl = MADCTL_MV;
            break;
        case 5:
            madctl = MADCTL_MV | MADCTL_MY;
            break;
        case 6:
            madctl = MADCTL_MV | MADCTL_MX;
            break;
        case 7:
            madctl = MADCTL_MV | MADCTL_MX | MADCTL_MY;
            break;
        default:
            return INVALID_ARGUMENT;
    }
    madctl |= is_bgr_ ? MADCTL_BGR : 0x00;

    err = cmd(CMD_MEM_ACCESS_CTL);
    ERROR_CHECK(err);
    err = data(&madctl, 1);
    ERROR_CHECK(err);
    return OK;
}

/**
 * @brief Sets the write window for the display
 * @param x starting `x` coordinate
 * @param y starting `y` coordinate
 * @param w width of the window
 * @param h height of the window
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::set_write_window(const u8 x, const u8 y, const u8 w, const u8 h) const {
    if (x > GC9A01_WIDTH || y > GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    if (x + w > GC9A01_WIDTH || y + h > GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }

    Error err;

    // Column address set
    u16 start = x;
    u16 end = x + w - 1;
    const u8 col[] = {
        static_cast<u8>(start >> 8),
        static_cast<u8>(start & 0xFF),
        static_cast<u8>(end >> 8),
        static_cast<u8>(end & 0xFF)
    };
    err = cmd(CMD_COLADDRSET);
    ERROR_CHECK(err);
    err = data(col, 4);
    ERROR_CHECK(err);

    // Row address set
    start = y;
    end = y + h - 1;
    const u8 row[] = {
        static_cast<u8>(start >> 8),
        static_cast<u8>(start & 0xFF),
        static_cast<u8>(end >> 8),
        static_cast<u8>(end & 0xFF)
    };
    err = cmd(CMD_ROWADDRSET);
    ERROR_CHECK(err);
    err = data(row, 4);
    ERROR_CHECK(err);
    err = cmd(CMD_MEMORY_WRITE);
    ERROR_CHECK(err);
    return OK;
}

/**
 * @brief Fill the screen with a `color`
 * TODO: Replace with `fill_rect`
 * @param color Color to fill the screen with
 * @return `OK` on success, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::fill(const Color color) const {
    Error err;
    LOG("Fill screen: Color(%d)", color.to_16bit());
    err = set_write_window(0, 0, GC9A01_WIDTH, GC9A01_HEIGHT);
    ERROR_CHECK(err);
    u16 color16 = color.to_16bit();
    u8 buf[2] = {static_cast<u8>(color16 >> 8), static_cast<u8>(color16 & 0xFF)};
    for (u32 i = 0; i < GC9A01_PIXELS; i++) {
        err = data(buf, 2);
        ERROR_CHECK(err);
    }
    return OK;
}

/**
 * @brief Set a pixel at `x`, `y` with a `color`
 * @param x `x` coordinate
 * @param y `y` coordinate
 * @param color Color of the pixel
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::set_pixel(const u16 x, const u16 y, const Color color) const {
    if (x >= GC9A01_WIDTH || y >= GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    Error err;
    err = set_write_window(x, y, 1, 1);
    ERROR_CHECK(err);
    u16 color16 = color.to_16bit();
    u8 buf[2] = {static_cast<u8>(color16 >> 8), static_cast<u8>(color16 & 0xFF)};
    err = data(buf, 2);
    ERROR_CHECK(err);
    return OK;
}


/**
 * @brief Clear the screen buffer
 * @return `OK` on success, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::clear() const {
    return fill(Color(0, 0, 0));
}

/**
 * @brief Draw a 16-bit color bitmap at `x`, `y`
 * 
 * @param x `x` coordinate
 * @param y `y` coordinate
 * @param w width of the bitmap
 * @param h height of the bitmap
 * @param bitmap 16-bit color array of the bitmap with size `w * h`
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::draw_bitmap(const u16 x, const u16 y, u16 w, u16 h, const u16* bitmap) const {
    if (bitmap == nullptr) {
        return INVALID_ARGUMENT;
    }
    if (x >= GC9A01_WIDTH || y >= GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    if (x + w > GC9A01_WIDTH || y + h > GC9A01_HEIGHT) {
        w = std::min(w, static_cast<u16>(GC9A01_WIDTH - x));
        h = std::min(h, static_cast<u16>(GC9A01_HEIGHT - y));
    }
    Error err;
    err = set_write_window(x, y, w, h);
    ERROR_CHECK(err);
    for (u32 i = 0; i < w * h; i++) {
        u16 color16 = bitmap[i];
        u8 buf[2] = {static_cast<u8>(color16 >> 8), static_cast<u8>(color16 & 0xFF)};
        err = data(buf, 2);
        ERROR_CHECK(err);
    }
    return OK;
}

/**
 * @brief Draw a horizontal line at `x`, `y` with a width of `w` and a `color`
 * @param x `x` coordinate
 * @param y `y` coordinate
 * @param w width
 * @param color Color of the line
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::draw_hline(const u16 x, const u16 y, u16 w, const Color color) const {
    if (x >= GC9A01_WIDTH || y >= GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    if (x + w > GC9A01_WIDTH) {
        w = GC9A01_WIDTH - x;
    }
    Error err;
    err = set_write_window(x, y, w, 1);
    ERROR_CHECK(err);
    u16 color16 = color.to_16bit();
    u8 buf[2] = {static_cast<u8>(color16 >> 8), static_cast<u8>(color16 & 0xFF)};
    for (u32 i = 0; i < w; i++) {
        err = data(buf, 2);
        ERROR_CHECK(err);
    }
    return OK;
}

/**
 * @brief Draw a vertical line at `x`, `y` with a height of `h` and a `color`
 * @param x `x` coordinate
 * @param y `y` coordinate
 * @param h height
 * @param color Color of the line
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::draw_vline(const u16 x, const u16 y, u16 h, const Color color) const {
    if (x >= GC9A01_WIDTH || y >= GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    if (y + h > GC9A01_HEIGHT) {
        h = GC9A01_HEIGHT - y;
    }
    Error err;
    err = set_write_window(x, y, 1, h);
    ERROR_CHECK(err);
    u16 color16 = color.to_16bit();
    u8 buf[2] = {static_cast<u8>(color16 >> 8), static_cast<u8>(color16 & 0xFF)};
    for (u32 i = 0; i < h; i++) {
        err = data(buf, 2);
        ERROR_CHECK(err);
    }
    return OK;
}

/**
 * @brief Draw a line from `x0`, `y0` to `x1`, `y1` with a `color`
 * **NOTE**: Not implemented
 * @param x0 `x0` coordinate
 * @param y0 `y0` coordinate
 * @param x1 `x1` coordinate
 * @param y1 `y1` coordinate
 * @param color Color of the line
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::draw_line(u16 x0, u16 y0, u16 x1, u16 y1, const Color color) const {
    if (x0 >= GC9A01_WIDTH || y0 >= GC9A01_HEIGHT || x1 >= GC9A01_WIDTH || y1 >= GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    // TODO: Implement
    // Error err;
    return OK;
}

/**
 * @brief Draw a rectangle at `x`, `y` with a width of `w`, a height of `h` and a `color`
 * 
 * @param x `x` coordinate
 * @param y `y` coordinate
 * @param w width of the rectangle
 * @param h height of the rectangle
 * @param color Color of the rectangle
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::draw_rect(u16 x, u16 y, u16 w, u16 h, const Color color) const {
    if (x >= GC9A01_WIDTH || y >= GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    if (x + w > GC9A01_WIDTH || y + h > GC9A01_HEIGHT) {
        w = std::min(w, static_cast<u16>(GC9A01_WIDTH - x));
        h = std::min(h, static_cast<u16>(GC9A01_HEIGHT - y));
    }
    Error err;
    err = draw_hline(x, y, w, color);
    ERROR_CHECK(err);
    err = draw_hline(x, y + h - 1, w, color);
    ERROR_CHECK(err);
    err = draw_vline(x, y, h, color);
    ERROR_CHECK(err);
    err = draw_vline(x + w - 1, y, h, color);
    ERROR_CHECK(err);
    return OK;
}

/**
 * @brief Draw a filled rectangle at `x`, `y` with a width of `w`, a height of `h` and a `color`
 * 
 * @param x `x` coordinate
 * @param y `y` coordinate
 * @param w width of the rectangle
 * @param h height of the rectangle
 * @param color Color of the rectangle
 * @return `OK` on success, `INVALID_ARGUMENT` if the arguments are invalid, else `SPI_TRANSMIT_ERROR`
 */
GC9A01::Error GC9A01::fill_rect(u16 x, u16 y, u16 w, u16 h, const Color color) const {
    if (x >= GC9A01_WIDTH || y >= GC9A01_HEIGHT) {
        return INVALID_ARGUMENT;
    }
    if (x + w > GC9A01_WIDTH || y + h > GC9A01_HEIGHT) {
        w = std::min(w, static_cast<u16>(GC9A01_WIDTH - x));
        h = std::min(h, static_cast<u16>(GC9A01_HEIGHT - y));
    }
    Error err;
    u16 color16 = color.to_16bit();
    u8 buf[2] = {static_cast<u8>(color16 >> 8), static_cast<u8>(color16 & 0xFF)};
    err = set_write_window(x, y, w, h);
    for (u32 i = 0; i < h * w; i++) {
        err = data(buf, 2);
        ERROR_CHECK(err);
    }
    return OK;
}

