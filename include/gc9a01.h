#include <cstdint>

typedef uint32_t u32;
typedef uint8_t u8;




/**
 * Error codes for the GC9A01 display driver
 */
typedef enum gc9a01_err_t {
    GC9A01_OK,
} gc9a01_err_t;

typedef struct color_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

typedef struct gc9a01_cmd_t {
    u8 cmd;
    u8 data[16];
    u8 datasize;
} gc9a01_cmd_t;


class GC9A01 {
public:
    GC9A01();
    ~GC9A01() = default;

    // NOTE: Maybe arguments needed
    gc9a01_err_t init();

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

private:
};
