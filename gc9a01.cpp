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
//...
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

#define NUM_INIT_COMMANDS 46

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

GC9A01::GC9A01() {
}

GC9A01::GC9A01(spi_device_handle_t spi, gpio_num_t mosi, gpio_num_t clk, gpio_num_t cs, gpio_num_t dc, gpio_num_t rst) : 
    spi_(spi), mosi_(mosi), clk_(clk), cs_(cs), dc_(dc), rst_(rst)
{

}


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

GC9A01::Error GC9A01::data(const u8* data, const u8 datasize) const { 
    esp_err_t err;
    spi_transaction_t t;

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
    // assert(err == ESP_OK);
    return err == ESP_OK ? OK : SPI_TRANSMIT_ERROR;
}

GC9A01::Error GC9A01::hard_reset() const {
    LOG("Hard reset");
    gpio_set_level(this->rst_, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(this->rst_, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return OK;
}

GC9A01::Error GC9A01::soft_reset() const {
    LOG("Soft reset");
    return cmd(CMD_SWRESET);
}

GC9A01::Error GC9A01::init() const
{
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
    return result;
}

