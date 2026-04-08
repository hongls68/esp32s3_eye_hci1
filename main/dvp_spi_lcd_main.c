#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "example_config.h"

#define APP_PI                          3.14159265358979323846f
#define APP_TAU                         (2.0f * APP_PI)
#define APP_FRAME_INTERVAL_MS           50
#define APP_LONG_PRESS_MS               700
#define APP_DEBOUNCE_MS                 30

#define GRID_W                          32
#define GRID_H                          32
#define GRID_SIZE                       (GRID_W * GRID_H)
#define PARTICLE_COUNT                  144
#define FLIP_BLEND                      0.90f
#define PARTICLE_RADIUS_PX              3
#define PRESSURE_ITERATIONS             10

typedef enum {
    APP_MODE_DIAL = 0,
    APP_MODE_FLUID,
} app_mode_t;

typedef enum {
    IMU_TYPE_NONE = 0,
    IMU_TYPE_MPU6050,
} imu_type_t;

typedef struct {
    uint16_t bg;
    uint16_t bezel;
    uint16_t primary;
    uint16_t secondary;
    uint16_t accent;
    uint16_t fluid_bg;
} ui_theme_t;

typedef struct {
    bool last_pressed;
    bool long_handled;
    int64_t pressed_at_us;
} button_state_t;

typedef struct {
    float x;
    float y;
    float vx;
    float vy;
} particle_t;

typedef struct {
    particle_t particles[PARTICLE_COUNT];
    float u[GRID_SIZE];
    float v[GRID_SIZE];
    float u_prev[GRID_SIZE];
    float v_prev[GRID_SIZE];
    float mass[GRID_SIZE];
    float divergence[GRID_SIZE];
    float pressure[GRID_SIZE];
    float pressure_tmp[GRID_SIZE];
    uint8_t preset;
} fluid_state_t;

typedef struct {
    bool available;
    imu_type_t type;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} motion_input_t;

typedef struct {
    esp_lcd_panel_handle_t panel;
    uint16_t *framebuf;
    app_mode_t mode;
    uint8_t theme_index;
    float tilt_x;
    float tilt_y;
    float fallback_phase;
    int64_t start_time_us;
    button_state_t button;
    motion_input_t motion;
    fluid_state_t fluid;
} app_state_t;

static const char *TAG = "eye_hci";
static app_state_t s_app;

static const ui_theme_t s_themes[] = {
    {.bg = 0x0000, .bezel = 0x18E3, .primary = 0xFFFF, .secondary = 0x00FF, .accent = 0xF800, .fluid_bg = 0x0000}, // 赛博朋克青色主题
    {.bg = 0x0000, .bezel = 0x18E3, .primary = 0xFFFF, .secondary = 0xF800, .accent = 0x00FF, .fluid_bg = 0x0000}, // 赛博朋克洋红色主题
    {.bg = 0x0000, .bezel = 0x18E3, .primary = 0xFFFF, .secondary = 0x7FFF, .accent = 0xF81F, .fluid_bg = 0x0000}, // 赛博朋克蓝紫色主题
};

static inline int clampi(int value, int min_v, int max_v)
{
    if (value < min_v) {
        return min_v;
    }
    if (value > max_v) {
        return max_v;
    }
    return value;
}

static inline float clampf(float value, float min_v, float max_v)
{
    if (value < min_v) {
        return min_v;
    }
    if (value > max_v) {
        return max_v;
    }
    return value;
}

static inline float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static inline int grid_index(int x, int y)
{
    return y * GRID_W + x;
}

static inline const ui_theme_t *current_theme(const app_state_t *app)
{
    return &s_themes[app->theme_index % (sizeof(s_themes) / sizeof(s_themes[0]))];
}

static inline float particle_to_grid_x(float nx)
{
    return clampf(nx * (GRID_W - 3) + 1.0f, 0.0f, (float)GRID_W - 1.001f);
}

static inline float particle_to_grid_y(float ny)
{
    return clampf(ny * (GRID_H - 3) + 1.0f, 0.0f, (float)GRID_H - 1.001f);
}

static void put_pixel(uint16_t *fb, int x, int y, uint16_t color)
{
    if (x < 0 || x >= APP_LCD_H_RES || y < 0 || y >= APP_LCD_V_RES) {
        return;
    }
    fb[y * APP_LCD_H_RES + x] = color;
}

static void clear_frame(uint16_t *fb, uint16_t color)
{
    for (int i = 0; i < APP_LCD_H_RES * APP_LCD_V_RES; i++) {
        fb[i] = color;
    }
}

static void fill_rect(uint16_t *fb, int x, int y, int w, int h, uint16_t color)
{
    int x0 = clampi(x, 0, APP_LCD_H_RES);
    int y0 = clampi(y, 0, APP_LCD_V_RES);
    int x1 = clampi(x + w, 0, APP_LCD_H_RES);
    int y1 = clampi(y + h, 0, APP_LCD_V_RES);

    for (int py = y0; py < y1; py++) {
        uint16_t *row = fb + py * APP_LCD_H_RES;
        for (int px = x0; px < x1; px++) {
            row[px] = color;
        }
    }
}

static void draw_line(uint16_t *fb, int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        put_pixel(fb, x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = err << 1;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void draw_circle(uint16_t *fb, int cx, int cy, int radius, uint16_t color)
{
    int x = radius;
    int y = 0;
    int err = 1 - x;

    while (x >= y) {
        put_pixel(fb, cx + x, cy + y, color);
        put_pixel(fb, cx + y, cy + x, color);
        put_pixel(fb, cx - y, cy + x, color);
        put_pixel(fb, cx - x, cy + y, color);
        put_pixel(fb, cx - x, cy - y, color);
        put_pixel(fb, cx - y, cy - x, color);
        put_pixel(fb, cx + y, cy - x, color);
        put_pixel(fb, cx + x, cy - y, color);
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

static void fill_circle(uint16_t *fb, int cx, int cy, int radius, uint16_t color)
{
    int rr = radius * radius;
    for (int y = -radius; y <= radius; y++) {
        int yy = y * y;
        for (int x = -radius; x <= radius; x++) {
            if ((x * x + yy) <= rr) {
                put_pixel(fb, cx + x, cy + y, color);
            }
        }
    }
}

static void draw_ring(uint16_t *fb, int cx, int cy, int radius, int thickness, uint16_t color)
{
    for (int i = 0; i < thickness; i++) {
        draw_circle(fb, cx, cy, radius - i, color);
    }
}

static void draw_status_bar(const app_state_t *app)
{
    const ui_theme_t *theme = current_theme(app);
    uint16_t *fb = app->framebuf;

    fill_rect(fb, 8, 8, 52, 10, theme->bezel);
    fill_rect(fb, 10, 10, app->mode == APP_MODE_DIAL ? 22 : 10, 6, theme->primary);
    fill_rect(fb, 34, 10, app->mode == APP_MODE_FLUID ? 22 : 10, 6, theme->secondary);

    uint16_t motion_color = app->motion.available ? rgb565(80, 255, 160) : rgb565(255, 180, 40);
    fill_rect(fb, APP_LCD_H_RES - 22, 10, 12, 12, motion_color);
    fill_rect(fb, APP_LCD_H_RES - 44, 10, 16, 12, theme->accent);
    for (int i = 0; i < 3; i++) {
        int w = 2 + i * 3;
        fill_rect(fb, APP_LCD_H_RES - 42 + i * 5, 18 - w, 3, w, theme->bg);
    }
}

static void draw_hand(uint16_t *fb, int cx, int cy, float angle, int length, uint16_t color)
{
    int x1 = cx + (int)(cosf(angle) * length);
    int y1 = cy + (int)(sinf(angle) * length);
    draw_line(fb, cx, cy, x1, y1, color);
    fill_circle(fb, x1, y1, 2, color);
}

static void lcd_display_init(esp_lcd_panel_handle_t *lcd_panel_hdl)
{
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_io_handle_t lcd_io_hdl = NULL;

    const gpio_config_t backlight_cfg = {
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_BACKLIGHT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&backlight_cfg));
    gpio_set_level(EXAMPLE_LCD_BACKLIGHT, 1);
    printf("L1\n");
    fflush(stdout);

    ESP_LOGI(TAG, "lcd: init spi bus");
    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = EXAMPLE_LCD_SPI_CLK,
        .mosi_io_num = EXAMPLE_LCD_SPI_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = APP_LCD_H_RES * APP_LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &bus_cfg, SPI_DMA_CH_AUTO));
    printf("L2\n");
    fflush(stdout);

    ESP_LOGI(TAG, "lcd: init panel io");
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = EXAMPLE_LCD_DC,
        .cs_gpio_num = EXAMPLE_LCD_SPI_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 2,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_cfg, &lcd_io_hdl));
    printf("L3\n");
    fflush(stdout);

    ESP_LOGI(TAG, "lcd: init st7789");
    const esp_lcd_panel_dev_config_t panel_dev_cfg = {
        .reset_gpio_num = EXAMPLE_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = APP_RGB565_BITS_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io_hdl, &panel_dev_cfg, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 20));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    printf("L4\n");
    fflush(stdout);

    gpio_set_level(EXAMPLE_LCD_BACKLIGHT, 0);
    ESP_LOGI(TAG, "lcd: backlight on");

    *lcd_panel_hdl = panel_handle;
}

static void button_init(button_state_t *button)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << EXAMPLE_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    memset(button, 0, sizeof(*button));
}

static esp_err_t motion_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, 50);
}

static esp_err_t motion_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(dev, payload, sizeof(payload), 50);
}

static bool motion_probe_mpu6050(motion_input_t *motion, uint16_t address)
{
    i2c_device_config_t dev_cfg = {
        .device_address = address,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(motion->bus, &dev_cfg, &dev) != ESP_OK) {
        return false;
    }

    uint8_t who_am_i = 0;
    if (motion_read_reg(dev, 0x75, &who_am_i, 1) != ESP_OK || (who_am_i != 0x68 && who_am_i != 0x70)) {
        i2c_master_bus_rm_device(dev);
        return false;
    }

    ESP_ERROR_CHECK(motion_write_reg(dev, 0x6B, 0x00));
    ESP_ERROR_CHECK(motion_write_reg(dev, 0x1C, 0x00));

    motion->dev = dev;
    motion->type = IMU_TYPE_MPU6050;
    motion->available = true;
    ESP_LOGI(TAG, "Detected MPU6050 at 0x%02X", address);
    return true;
}

static void motion_input_init(motion_input_t *motion)
{
    memset(motion, 0, sizeof(*motion));

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = EXAMPLE_SENSOR_I2C_SDA_IO,
        .scl_io_num = EXAMPLE_SENSOR_I2C_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &motion->bus) != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed, using fallback motion");
        return;
    }

    if (motion_probe_mpu6050(motion, 0x68) || motion_probe_mpu6050(motion, 0x69)) {
        return;
    }

    ESP_LOGW(TAG, "No accelerometer detected on I2C, using fallback motion");
}

static bool motion_input_sample(motion_input_t *motion, float *tilt_x, float *tilt_y)
{
    if (!motion->available || motion->type != IMU_TYPE_MPU6050) {
        return false;
    }

    uint8_t raw[6] = {0};
    if (motion_read_reg(motion->dev, 0x3B, raw, sizeof(raw)) != ESP_OK) {
        motion->available = false;
        ESP_LOGW(TAG, "Accelerometer read failed, fallback enabled");
        return false;
    }

    const int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    const int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);

    *tilt_x = clampf((float)ay / 16384.0f, -1.2f, 1.2f);
    *tilt_y = clampf(-(float)ax / 16384.0f, -1.2f, 1.2f);
    return true;
}

static void motion_update(app_state_t *app, float dt)
{
    float tx = 0.0f;
    float ty = 0.0f;

    if (motion_input_sample(&app->motion, &tx, &ty)) {
        app->tilt_x = lerpf(app->tilt_x, tx, 0.2f);
        app->tilt_y = lerpf(app->tilt_y, ty, 0.2f);
        return;
    }

    app->fallback_phase += dt;
    tx = 0.65f * sinf(app->fallback_phase * 0.9f);
    ty = 0.45f * cosf(app->fallback_phase * 1.3f);
    app->tilt_x = lerpf(app->tilt_x, tx, 0.06f);
    app->tilt_y = lerpf(app->tilt_y, ty, 0.06f);
}

static void fluid_apply_preset(fluid_state_t *fluid, uint8_t preset)
{
    fluid->preset = preset % 3;
    memset(fluid->u, 0, sizeof(fluid->u));
    memset(fluid->v, 0, sizeof(fluid->v));
    memset(fluid->u_prev, 0, sizeof(fluid->u_prev));
    memset(fluid->v_prev, 0, sizeof(fluid->v_prev));
    memset(fluid->mass, 0, sizeof(fluid->mass));
    memset(fluid->divergence, 0, sizeof(fluid->divergence));
    memset(fluid->pressure, 0, sizeof(fluid->pressure));
    memset(fluid->pressure_tmp, 0, sizeof(fluid->pressure_tmp));

    for (int i = 0; i < PARTICLE_COUNT; i++) {
        const float t = (float)i / (float)PARTICLE_COUNT;
        particle_t *p = &fluid->particles[i];
        if (fluid->preset == 0) {
            float angle = t * APP_TAU;
            float radius = 0.16f + 0.10f * ((float)(i % 9) / 8.0f);
            p->x = 0.5f + cosf(angle) * radius;
            p->y = 0.58f + sinf(angle) * radius;
            p->vx = -sinf(angle) * 0.18f;
            p->vy = cosf(angle) * 0.18f;
        } else if (fluid->preset == 1) {
            float side = (i & 1) ? 0.34f : 0.66f;
            float row = (float)(i / 2) / ((float)PARTICLE_COUNT / 2.0f);
            p->x = side + 0.08f * sinf(row * APP_TAU * 2.0f);
            p->y = 0.24f + row * 0.50f;
            p->vx = (i & 1) ? -0.14f : 0.14f;
            p->vy = 0.04f * cosf(row * APP_TAU * 3.0f);
        } else {
            float angle = t * APP_TAU;
            float radius = 0.24f;
            p->x = 0.5f + cosf(angle) * radius;
            p->y = 0.52f + sinf(angle) * radius * 0.65f;
            p->vx = cosf(angle) * 0.10f;
            p->vy = -0.28f * fabsf(sinf(angle));
        }
    }
}

static void fluid_impulse(fluid_state_t *fluid, float gx, float gy)
{
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        particle_t *p = &fluid->particles[i];
        float dx = p->x - 0.5f;
        float dy = p->y - 0.5f;
        float dist = sqrtf(dx * dx + dy * dy) + 0.001f;
        float gain = clampf(0.24f - dist, 0.0f, 0.24f) * 8.0f;
        p->vx += gx * gain;
        p->vy += gy * gain;
    }
}

static void fluid_step(fluid_state_t *fluid, float dt, float gravity_x, float gravity_y)
{
    memset(fluid->u, 0, sizeof(fluid->u));
    memset(fluid->v, 0, sizeof(fluid->v));
    memset(fluid->mass, 0, sizeof(fluid->mass));
    memset(fluid->pressure, 0, sizeof(fluid->pressure));
    memset(fluid->pressure_tmp, 0, sizeof(fluid->pressure_tmp));

    for (int i = 0; i < PARTICLE_COUNT; i++) {
        const particle_t *p = &fluid->particles[i];
        float gx = particle_to_grid_x(p->x);
        float gy = particle_to_grid_y(p->y);
        int x0 = (int)floorf(gx);
        int y0 = (int)floorf(gy);
        int x1 = clampi(x0 + 1, 0, GRID_W - 1);
        int y1 = clampi(y0 + 1, 0, GRID_H - 1);
        float tx = gx - (float)x0;
        float ty = gy - (float)y0;

        float w00 = (1.0f - tx) * (1.0f - ty);
        float w10 = tx * (1.0f - ty);
        float w01 = (1.0f - tx) * ty;
        float w11 = tx * ty;

        int i00 = grid_index(x0, y0);
        int i10 = grid_index(x1, y0);
        int i01 = grid_index(x0, y1);
        int i11 = grid_index(x1, y1);

        fluid->mass[i00] += w00;
        fluid->mass[i10] += w10;
        fluid->mass[i01] += w01;
        fluid->mass[i11] += w11;

        fluid->u[i00] += p->vx * w00;
        fluid->u[i10] += p->vx * w10;
        fluid->u[i01] += p->vx * w01;
        fluid->u[i11] += p->vx * w11;

        fluid->v[i00] += p->vy * w00;
        fluid->v[i10] += p->vy * w10;
        fluid->v[i01] += p->vy * w01;
        fluid->v[i11] += p->vy * w11;
    }

    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            int idx = grid_index(x, y);
            if (fluid->mass[idx] > 0.0001f) {
                fluid->u[idx] /= fluid->mass[idx];
                fluid->v[idx] /= fluid->mass[idx];
            }
            fluid->u_prev[idx] = fluid->u[idx];
            fluid->v_prev[idx] = fluid->v[idx];
        }
    }

    for (int y = 1; y < GRID_H - 1; y++) {
        for (int x = 1; x < GRID_W - 1; x++) {
            int idx = grid_index(x, y);
            fluid->u[idx] += gravity_x * dt;
            fluid->v[idx] += gravity_y * dt;
        }
    }

    for (int y = 1; y < GRID_H - 1; y++) {
        for (int x = 1; x < GRID_W - 1; x++) {
            int idx = grid_index(x, y);
            fluid->divergence[idx] = 0.5f * ((fluid->u[grid_index(x + 1, y)] - fluid->u[grid_index(x - 1, y)]) +
                                             (fluid->v[grid_index(x, y + 1)] - fluid->v[grid_index(x, y - 1)]));
        }
    }

    float *pressure_read = fluid->pressure;
    float *pressure_write = fluid->pressure_tmp;
    for (int iter = 0; iter < PRESSURE_ITERATIONS; iter++) {
        for (int y = 1; y < GRID_H - 1; y++) {
            for (int x = 1; x < GRID_W - 1; x++) {
                int idx = grid_index(x, y);
                pressure_write[idx] = (fluid->divergence[idx] + pressure_read[grid_index(x - 1, y)] + pressure_read[grid_index(x + 1, y)] +
                                       pressure_read[grid_index(x, y - 1)] + pressure_read[grid_index(x, y + 1)]) * 0.25f;
            }
        }
        float *tmp = pressure_read;
        pressure_read = pressure_write;
        pressure_write = tmp;
    }
    if (pressure_read != fluid->pressure) {
        memcpy(fluid->pressure, pressure_read, sizeof(fluid->pressure));
    }

    for (int y = 1; y < GRID_H - 1; y++) {
        for (int x = 1; x < GRID_W - 1; x++) {
            int idx = grid_index(x, y);
            fluid->u[idx] -= 0.5f * (fluid->pressure[grid_index(x + 1, y)] - fluid->pressure[grid_index(x - 1, y)]);
            fluid->v[idx] -= 0.5f * (fluid->pressure[grid_index(x, y + 1)] - fluid->pressure[grid_index(x, y - 1)]);
        }
    }

    for (int x = 0; x < GRID_W; x++) {
        fluid->u[grid_index(x, 0)] = 0.0f;
        fluid->u[grid_index(x, GRID_H - 1)] = 0.0f;
        fluid->v[grid_index(x, 0)] = 0.0f;
        fluid->v[grid_index(x, GRID_H - 1)] = 0.0f;
    }
    for (int y = 0; y < GRID_H; y++) {
        fluid->u[grid_index(0, y)] = 0.0f;
        fluid->u[grid_index(GRID_W - 1, y)] = 0.0f;
        fluid->v[grid_index(0, y)] = 0.0f;
        fluid->v[grid_index(GRID_W - 1, y)] = 0.0f;
    }

    for (int i = 0; i < PARTICLE_COUNT; i++) {
        particle_t *p = &fluid->particles[i];
        float gx = particle_to_grid_x(p->x);
        float gy = particle_to_grid_y(p->y);
        int x0 = (int)floorf(gx);
        int y0 = (int)floorf(gy);
        int x1 = clampi(x0 + 1, 0, GRID_W - 1);
        int y1 = clampi(y0 + 1, 0, GRID_H - 1);
        float tx = gx - (float)x0;
        float ty = gy - (float)y0;

        float w00 = (1.0f - tx) * (1.0f - ty);
        float w10 = tx * (1.0f - ty);
        float w01 = (1.0f - tx) * ty;
        float w11 = tx * ty;

        int i00 = grid_index(x0, y0);
        int i10 = grid_index(x1, y0);
        int i01 = grid_index(x0, y1);
        int i11 = grid_index(x1, y1);

        float pic_vx = fluid->u[i00] * w00 + fluid->u[i10] * w10 + fluid->u[i01] * w01 + fluid->u[i11] * w11;
        float pic_vy = fluid->v[i00] * w00 + fluid->v[i10] * w10 + fluid->v[i01] * w01 + fluid->v[i11] * w11;
        float delta_vx = (fluid->u[i00] - fluid->u_prev[i00]) * w00 + (fluid->u[i10] - fluid->u_prev[i10]) * w10 +
                         (fluid->u[i01] - fluid->u_prev[i01]) * w01 + (fluid->u[i11] - fluid->u_prev[i11]) * w11;
        float delta_vy = (fluid->v[i00] - fluid->v_prev[i00]) * w00 + (fluid->v[i10] - fluid->v_prev[i10]) * w10 +
                         (fluid->v[i01] - fluid->v_prev[i01]) * w01 + (fluid->v[i11] - fluid->v_prev[i11]) * w11;

        float flip_vx = p->vx + delta_vx;
        float flip_vy = p->vy + delta_vy;

        p->vx = lerpf(pic_vx, flip_vx, FLIP_BLEND) * 0.995f;
        p->vy = lerpf(pic_vy, flip_vy, FLIP_BLEND) * 0.995f;

        p->x += p->vx * dt;
        p->y += p->vy * dt;

        if (p->x < 0.03f) {
            p->x = 0.03f;
            p->vx *= -0.35f;
        } else if (p->x > 0.97f) {
            p->x = 0.97f;
            p->vx *= -0.35f;
        }
        if (p->y < 0.03f) {
            p->y = 0.03f;
            p->vy *= -0.35f;
        } else if (p->y > 0.97f) {
            p->y = 0.97f;
            p->vy *= -0.35f;
        }
    }
}

static void draw_dial(app_state_t *app, float uptime_s)
{
    const ui_theme_t *theme = current_theme(app);
    uint16_t *fb = app->framebuf;
    clear_frame(fb, theme->bg);

    const int cx = APP_LCD_H_RES / 2;
    const int cy = APP_LCD_V_RES / 2;
    const int outer_r = 106;
    const int inner_r = 88;

    fill_circle(fb, cx, cy, 112, theme->bezel);
    fill_circle(fb, cx, cy, 102, theme->bg);
    draw_ring(fb, cx, cy, outer_r, 3, theme->primary);
    draw_ring(fb, cx, cy, inner_r, 2, theme->secondary);

    for (int i = 0; i < 60; i++) {
        float angle = ((float)i / 60.0f) * APP_TAU - APP_PI / 2.0f;
        int tick_outer = outer_r - 4;
        int tick_inner = (i % 5 == 0) ? outer_r - 20 : outer_r - 11;
        uint16_t color = (i % 5 == 0) ? theme->accent : theme->secondary;
        int x0 = cx + (int)(cosf(angle) * tick_inner);
        int y0 = cy + (int)(sinf(angle) * tick_inner);
        int x1 = cx + (int)(cosf(angle) * tick_outer);
        int y1 = cy + (int)(sinf(angle) * tick_outer);
        draw_line(fb, x0, y0, x1, y1, color);
    }

    float tilt_angle = atan2f(app->tilt_y, app->tilt_x + 0.0001f);
    float tilt_mag = clampf(sqrtf(app->tilt_x * app->tilt_x + app->tilt_y * app->tilt_y), 0.0f, 1.0f);
    float second_angle = fmodf(uptime_s * 0.80f, 1.0f) * APP_TAU - APP_PI / 2.0f;
    float minute_angle = fmodf(uptime_s * 0.16f + tilt_mag * 0.08f, 1.0f) * APP_TAU - APP_PI / 2.0f;
    float hour_angle = fmodf(uptime_s * 0.04f + (float)app->theme_index * 0.12f, 1.0f) * APP_TAU - APP_PI / 2.0f;

    draw_hand(fb, cx, cy, hour_angle, 46, theme->primary);
    draw_hand(fb, cx, cy, minute_angle, 72, theme->secondary);
    draw_hand(fb, cx, cy, second_angle, 86, theme->accent);

    int bubble_x = cx + (int)(app->tilt_x * 32.0f);
    int bubble_y = cy + (int)(app->tilt_y * 32.0f);
    draw_ring(fb, cx, cy, 34, 1, theme->secondary);
    fill_circle(fb, bubble_x, bubble_y, 9, theme->accent);
    draw_line(fb, cx, cy, cx + (int)(cosf(tilt_angle) * 34.0f), cy + (int)(sinf(tilt_angle) * 34.0f), theme->primary);
    fill_circle(fb, cx, cy, 6, theme->primary);

    for (int i = 0; i < 18; i++) {
        float angle = ((float)i / 18.0f) * APP_TAU + uptime_s * 0.08f;
        int radius = 16 + (i % 3) * 6;
        int px = cx + (int)(cosf(angle) * radius);
        int py = cy + (int)(sinf(angle) * radius);
        put_pixel(fb, px, py, theme->secondary);
    }

    draw_status_bar(app);
}

static void draw_fluid(app_state_t *app)
{
    const ui_theme_t *theme = current_theme(app);
    uint16_t *fb = app->framebuf;
    clear_frame(fb, theme->fluid_bg);

    const int cx = APP_LCD_H_RES / 2;
    const int cy = APP_LCD_V_RES / 2;
    fill_circle(fb, cx, cy, 112, theme->bezel);
    fill_circle(fb, cx, cy, 101, theme->fluid_bg);
    draw_ring(fb, cx, cy, 106, 3, theme->primary);
    draw_ring(fb, cx, cy, 92, 2, theme->secondary);

    for (int i = 0; i < PARTICLE_COUNT; i++) {
        const particle_t *p = &app->fluid.particles[i];
        int sx = 18 + (int)(p->x * 204.0f);
        int sy = 18 + (int)(p->y * 204.0f);
        float speed = clampf(sqrtf(p->vx * p->vx + p->vy * p->vy) * 4.0f, 0.0f, 1.0f);
        uint16_t c0 = speed > 0.72f ? theme->accent : theme->secondary;
        uint16_t c1 = speed > 0.30f ? theme->primary : theme->secondary;
        fill_circle(fb, sx, sy, PARTICLE_RADIUS_PX, c1);
        put_pixel(fb, sx, sy, c0);
        put_pixel(fb, sx + 1, sy, c0);
        put_pixel(fb, sx, sy + 1, c0);
    }

    int gx = cx + (int)(app->tilt_x * 44.0f);
    int gy = cy + (int)(app->tilt_y * 44.0f);
    draw_line(fb, cx, cy, gx, gy, theme->accent);
    fill_circle(fb, gx, gy, 4, theme->accent);
    fill_circle(fb, cx, cy, 3, theme->primary);

    draw_status_bar(app);
}

static bool g_fluid_paused = false;

static void app_on_short_press(app_state_t *app)
{
    // 切换颜色主题
    app->theme_index = (app->theme_index + 1) % (sizeof(s_themes) / sizeof(s_themes[0]));
    ESP_LOGI(TAG, "Color theme -> %u", app->theme_index);
}

static void app_on_long_press(app_state_t *app)
{
    // 暂停/重置流体模拟
    g_fluid_paused = !g_fluid_paused;
    if (g_fluid_paused) {
        ESP_LOGI(TAG, "Fluid simulation paused");
    } else {
        // 重置流体
        fluid_apply_preset(&app->fluid, app->fluid.preset + 1);
        fluid_impulse(&app->fluid, app->tilt_x * 1.5f, app->tilt_y * 1.5f - 0.4f);
        ESP_LOGI(TAG, "Fluid simulation reset and resumed");
    }
}

static void button_poll(app_state_t *app)
{
    button_state_t *button = &app->button;
    bool pressed = gpio_get_level(EXAMPLE_BUTTON_GPIO) == EXAMPLE_BUTTON_ACTIVE_LEVEL;
    int64_t now_us = esp_timer_get_time();

    if (pressed && !button->last_pressed) {
        button->pressed_at_us = now_us;
        button->long_handled = false;
    }

    if (pressed && !button->long_handled) {
        int64_t held_ms = (now_us - button->pressed_at_us) / 1000;
        if (held_ms >= APP_LONG_PRESS_MS) {
            app_on_long_press(app);
            button->long_handled = true;
        }
    }

    if (!pressed && button->last_pressed) {
        int64_t held_ms = (now_us - button->pressed_at_us) / 1000;
        if (!button->long_handled && held_ms >= APP_DEBOUNCE_MS) {
            app_on_short_press(app);
        }
    }

    button->last_pressed = pressed;
}

static lv_obj_t *g_canvas;
static lv_color_t *g_canvas_buf;

void app_main(void)
{
    app_state_t *app = &s_app;
    memset(app, 0, sizeof(*app));
    app->mode = APP_MODE_FLUID;
    app->theme_index = 0;
    app->start_time_us = esp_timer_get_time();

    printf("A1\n");
    fflush(stdout);
    lcd_display_init(&app->panel);
    printf("A2\n");
    fflush(stdout);
    button_init(&app->button);
    printf("A3\n");
    fflush(stdout);
    motion_input_init(&app->motion);
    printf("A4\n");
    fflush(stdout);

    // 初始化LVGL
    lvgl_port_cfg_t lvgl_cfg = LVGL_PORT_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    // 添加显示
    lvgl_port_display_cfg_t disp_cfg = LVGL_PORT_DISPLAY_DEFAULT_CONFIG(app->panel);
    disp_cfg.buffer_size = APP_LCD_H_RES * APP_LCD_V_RES * sizeof(uint16_t);
    disp_cfg.double_buffer = true;
    ESP_ERROR_CHECK(lvgl_port_add_disp(&disp_cfg));

    // 创建Canvas用于流体绘制
    g_canvas_buf = heap_caps_malloc(APP_LCD_H_RES * APP_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (g_canvas_buf == NULL) {
        ESP_LOGE(TAG, "Canvas buffer allocation failed");
        return;
    }

    g_canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(g_canvas, g_canvas_buf, APP_LCD_H_RES, APP_LCD_V_RES, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_size(g_canvas, APP_LCD_H_RES, APP_LCD_V_RES);
    lv_obj_center(g_canvas);

    // 初始化流体
    fluid_apply_preset(&app->fluid, 0);
    printf("A6\n");
    fflush(stdout);

    // 创建赛博朋克风格的HUD元素
    lv_obj_t *hud_frame = lv_obj_create(lv_scr_act());
    lv_obj_set_size(hud_frame, APP_LCD_H_RES - 20, APP_LCD_V_RES - 20);
    lv_obj_center(hud_frame);
    lv_obj_set_style_bg_color(hud_frame, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_color(hud_frame, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_border_width(hud_frame, 1, 0);
    lv_obj_set_style_border_opa(hud_frame, 150, 0);
    lv_obj_set_style_radius(hud_frame, 8, 0);
    lv_obj_set_style_shadow_color(hud_frame, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_shadow_width(hud_frame, 4, 0);
    lv_obj_set_style_shadow_opa(hud_frame, 100, 0);

    // 添加G-Sensor数据显示
    lv_obj_t *sensor_label = lv_label_create(lv_scr_act());
    lv_label_set_text(sensor_label, "G-Sensor: 0.00, 0.00");
    lv_obj_set_pos(sensor_label, 10, 10);
    lv_obj_set_style_text_color(sensor_label, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_bg_color(sensor_label, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(sensor_label, 100, 0);

    int64_t last_frame_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Project ready: short press toggles color theme, long press pauses/resets fluid");

    while (true) {
        int64_t frame_begin_us = esp_timer_get_time();
        float dt = (float)(frame_begin_us - last_frame_us) / 1000000.0f;
        last_frame_us = frame_begin_us;
        dt = clampf(dt, 0.010f, 0.040f);

        button_poll(app);
        motion_update(app, dt);

        // 更新G-Sensor数据显示
        char sensor_text[32];
        sprintf(sensor_text, "G-Sensor: %.2f, %.2f", app->tilt_x, app->tilt_y);
        lv_label_set_text(sensor_label, sensor_text);

        // 流体模拟和绘制
        if (!g_fluid_paused) {
            float gx = app->tilt_x * 9.5f;
            float gy = app->tilt_y * 9.5f + 2.0f;
            fluid_step(&app->fluid, dt, gx, gy);
        }
        
        // 清除canvas
        lv_canvas_fill_bg(g_canvas, lv_color_hex(0x000000), LV_OPA_COVER);
        
        // 绘制流体粒子
        const ui_theme_t *theme = current_theme(app);
        for (int i = 0; i < PARTICLE_COUNT; i++) {
            const particle_t *p = &app->fluid.particles[i];
            int sx = 18 + (int)(p->x * 204.0f);
            int sy = 18 + (int)(p->y * 204.0f);
            float speed = clampf(sqrtf(p->vx * p->vx + p->vy * p->vy) * 4.0f, 0.0f, 1.0f);
            
            // 绘制粒子拖尾
            for (int j = 1; j <= 3; j++) {
                float t = (float)j / 3.0f;
                int tx = sx - (int)(p->vx * t * 10.0f);
                int ty = sy - (int)(p->vy * t * 10.0f);
                lv_color_t color = lv_color_hex(speed > 0.72f ? theme->accent : theme->secondary);
                lv_canvas_draw_circle(g_canvas, tx, ty, PARTICLE_RADIUS_PX - j + 1, color, LV_OPA_50, color);
            }
            
            // 绘制粒子主体
            lv_color_t color = lv_color_hex(speed > 0.30f ? theme->primary : theme->secondary);
            lv_canvas_draw_circle(g_canvas, sx, sy, PARTICLE_RADIUS_PX, color, LV_OPA_COVER, color);
        }

        // 绘制重力方向指示器
        int cx = APP_LCD_H_RES / 2;
        int cy = APP_LCD_V_RES / 2;
        int gx_pos = cx + (int)(app->tilt_x * 44.0f);
        int gy_pos = cy + (int)(app->tilt_y * 44.0f);
        lv_canvas_draw_line(g_canvas, cx, cy, gx_pos, gy_pos, lv_color_hex(theme->accent), 2);
        lv_canvas_draw_circle(g_canvas, gx_pos, gy_pos, 4, lv_color_hex(theme->accent), LV_OPA_COVER, lv_color_hex(theme->accent));
        lv_canvas_draw_circle(g_canvas, cx, cy, 3, lv_color_hex(theme->primary), LV_OPA_COVER, lv_color_hex(theme->primary));

        // 刷新LVGL显示
        lv_timer_handler();

        int64_t spent_ms = (esp_timer_get_time() - frame_begin_us) / 1000;
        if (spent_ms < APP_FRAME_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(APP_FRAME_INTERVAL_MS - spent_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
