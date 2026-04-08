#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/lock.h>

#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "sdkconfig.h"

#include "example_config.h"

#define APP_PI                           3.14159265358979323846f
#define APP_LVGL_DRAW_BUF_LINES          24
#define APP_LVGL_TICK_PERIOD_MS          2
#define APP_LVGL_TASK_STACK_SIZE         (4 * 1024)
#define APP_LVGL_TASK_PRIORITY           1
#define APP_LVGL_TASK_MAX_DELAY_MS       500
#define APP_LVGL_TASK_MIN_DELAY_MS       (1000 / CONFIG_FREERTOS_HZ)

#define APP_DIAL_SIZE                    204
#define APP_DIAL_RADIUS                  (APP_DIAL_SIZE / 2)
#define APP_TICK_COUNT                   12
#define APP_LCD_BACKLIGHT_TIMER          LEDC_TIMER_1
#define APP_LCD_BACKLIGHT_CHANNEL        LEDC_CHANNEL_0
#define APP_LCD_BACKLIGHT_FREQ_HZ        5000
#define APP_LCD_BACKLIGHT_DUTY_MAX       1023
#define APP_HAND_ANIM_MS                 180

#define APP_SIM_START_HOUR               10
#define APP_SIM_START_MINUTE             8
#define APP_SIM_START_SECOND             36

typedef struct {
    bool use_system_time;
    int64_t boot_us;
    uint32_t base_seconds;
} clock_model_t;

typedef struct {
    lv_obj_t *line;
    lv_obj_t *counterweight;
    lv_point_precise_t points[2];
    int32_t current_rotation;
    int32_t head_len;
    int32_t tail_len;
    int32_t counterweight_size;
} hand_widget_t;

typedef struct {
    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_io_handle_t io;
    lv_display_t *display;
    lv_obj_t *face;
    lv_obj_t *info_label;
    lv_obj_t *ticks[APP_TICK_COUNT];
    lv_point_precise_t tick_points[APP_TICK_COUNT][2];
    lv_obj_t *numerals[4];
    hand_widget_t hour_hand;
    hand_widget_t minute_hand;
    hand_widget_t second_hand;
    lv_obj_t *center_outer;
    lv_obj_t *center_inner;
    lv_timer_t *clock_timer;
    clock_model_t clock;
} app_state_t;

static const char *TAG = "eye_clock";
static _lock_t s_lvgl_lock;
static app_state_t s_app;

static inline int32_t normalize_rotation_tenths(int32_t value)
{
    int32_t normalized = value % 3600;
    if (normalized < 0) {
        normalized += 3600;
    }
    return normalized;
}

static inline int32_t forward_rotation_delta(int32_t current, int32_t target)
{
    return normalize_rotation_tenths(target - current);
}

static void lcd_backlight_init(void)
{
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = APP_LCD_BACKLIGHT_TIMER,
        .freq_hz = APP_LCD_BACKLIGHT_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const ledc_channel_config_t channel_cfg = {
        .gpio_num = EXAMPLE_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = APP_LCD_BACKLIGHT_CHANNEL,
        .timer_sel = APP_LCD_BACKLIGHT_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = true,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void lcd_backlight_set_percent(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    uint32_t duty = (APP_LCD_BACKLIGHT_DUTY_MAX * percent) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, APP_LCD_BACKLIGHT_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, APP_LCD_BACKLIGHT_CHANNEL));
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    LV_UNUSED(panel_io);
    LV_UNUSED(edata);
    lv_display_t *display = (lv_display_t *)user_ctx;
    lv_display_flush_ready(display);
    return false;
}

static void lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = lv_display_get_user_data(display);
    lv_draw_sw_rgb565_swap(px_map, (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
}

static void lvgl_tick_cb(void *arg)
{
    LV_UNUSED(arg);
    lv_tick_inc(APP_LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *arg)
{
    LV_UNUSED(arg);
    while (true) {
        _lock_acquire(&s_lvgl_lock);
        uint32_t delay_ms = lv_timer_handler();
        _lock_release(&s_lvgl_lock);

        if (delay_ms < APP_LVGL_TASK_MIN_DELAY_MS) {
            delay_ms = APP_LVGL_TASK_MIN_DELAY_MS;
        }
        if (delay_ms > APP_LVGL_TASK_MAX_DELAY_MS) {
            delay_ms = APP_LVGL_TASK_MAX_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void lcd_display_init(app_state_t *app)
{
    lcd_backlight_init();
    lcd_backlight_set_percent(0);

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = EXAMPLE_LCD_SPI_CLK,
        .mosi_io_num = EXAMPLE_LCD_SPI_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = APP_LCD_H_RES * APP_LVGL_DRAW_BUF_LINES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &bus_cfg, SPI_DMA_CH_AUTO));

    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = EXAMPLE_LCD_DC,
        .cs_gpio_num = EXAMPLE_LCD_SPI_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 2,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_cfg, &app->io));

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = EXAMPLE_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = APP_RGB565_BITS_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(app->io, &panel_cfg, &app->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(app->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(app->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(app->panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(app->panel, true));

    vTaskDelay(pdMS_TO_TICKS(20));
    lcd_backlight_set_percent(100);
}

static void lvgl_init_display(app_state_t *app)
{
    lv_init();
    app->display = lv_display_create(APP_LCD_H_RES, APP_LCD_V_RES);

    size_t draw_buf_size = APP_LCD_H_RES * APP_LVGL_DRAW_BUF_LINES * sizeof(uint16_t);
    void *buf1 = spi_bus_dma_memory_alloc(EXAMPLE_LCD_SPI_NUM, draw_buf_size, 0);
    void *buf2 = spi_bus_dma_memory_alloc(EXAMPLE_LCD_SPI_NUM, draw_buf_size, 0);
    assert(buf1 && buf2);

    lv_display_set_buffers(app->display, buf1, buf2, draw_buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(app->display, app->panel);
    lv_display_set_color_format(app->display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(app->display, lvgl_flush_cb);

    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(app->io, &cbs, app->display));

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, APP_LVGL_TICK_PERIOD_MS * 1000));
}

static void clock_model_init(clock_model_t *clock)
{
    memset(clock, 0, sizeof(*clock));

    time_t now = time(NULL);
    struct tm now_tm = {0};
    if (now > 1704067200 && localtime_r(&now, &now_tm) != NULL) {
        int year = now_tm.tm_year + 1900;
        if (year >= 2024 && year < 2100) {
            clock->use_system_time = true;
            return;
        }
    }

    clock->use_system_time = false;
    clock->boot_us = esp_timer_get_time();
    clock->base_seconds = APP_SIM_START_HOUR * 3600U + APP_SIM_START_MINUTE * 60U + APP_SIM_START_SECOND;
}

static float clock_model_get_seconds_today(const clock_model_t *clock)
{
    if (clock->use_system_time) {
        time_t now = time(NULL);
        struct tm now_tm = {0};
        if (localtime_r(&now, &now_tm) != NULL) {
            return (float)(now_tm.tm_hour * 3600 + now_tm.tm_min * 60 + now_tm.tm_sec);
        }
    }

    double elapsed = (double)(esp_timer_get_time() - clock->boot_us) / 1000000.0;
    double simulated = fmod((double)clock->base_seconds + elapsed, 86400.0);
    if (simulated < 0.0) {
        simulated += 86400.0;
    }
    return (float)simulated;
}

static void clock_model_get_angles(const clock_model_t *clock, int32_t *hour_tenths, int32_t *minute_tenths, int32_t *second_tenths)
{
    float seconds_today = clock_model_get_seconds_today(clock);
    float seconds = fmodf(seconds_today, 60.0f);
    float minutes = fmodf(seconds_today / 60.0f, 60.0f);
    float hours = fmodf(seconds_today / 3600.0f, 12.0f);

    *second_tenths = (int32_t)lroundf(seconds * 60.0f);
    *minute_tenths = (int32_t)lroundf(minutes * 60.0f);
    *hour_tenths = (int32_t)lroundf(hours * 300.0f);
}

static void clock_model_format(const clock_model_t *clock, char *buf, size_t buf_len)
{
    float seconds_today = clock_model_get_seconds_today(clock);
    int total_seconds = (int)seconds_today;
    int hours = (total_seconds / 3600) % 24;
    int minutes = (total_seconds / 60) % 60;
    int seconds = total_seconds % 60;

    snprintf(buf, buf_len, "%s  %02d:%02d:%02d", clock->use_system_time ? "SYS" : "SIM", hours, minutes, seconds);
}

static void create_tick(app_state_t *app, lv_obj_t *parent, uint32_t index)
{
    const bool major = (index % 3U) == 0U;
    const float angle = ((float)index * 30.0f - 90.0f) * (APP_PI / 180.0f);
    const float outer_radius = 92.0f;
    const float inner_radius = major ? 70.0f : 80.0f;

    app->tick_points[index][0].x = APP_DIAL_RADIUS + cosf(angle) * inner_radius;
    app->tick_points[index][0].y = APP_DIAL_RADIUS + sinf(angle) * inner_radius;
    app->tick_points[index][1].x = APP_DIAL_RADIUS + cosf(angle) * outer_radius;
    app->tick_points[index][1].y = APP_DIAL_RADIUS + sinf(angle) * outer_radius;

    lv_obj_t *line = lv_line_create(parent);
    app->ticks[index] = line;
    lv_obj_set_size(line, APP_DIAL_SIZE, APP_DIAL_SIZE);
    lv_obj_set_pos(line, 0, 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_set_style_pad_all(line, 0, 0);
    lv_obj_set_style_line_width(line, major ? 4 : 2, 0);
    lv_obj_set_style_line_color(line, lv_color_hex(major ? 0xD4D8DE : 0x4F545C), 0);
    lv_obj_set_style_line_opa(line, major ? LV_OPA_90 : LV_OPA_70, 0);
    lv_obj_set_style_line_rounded(line, true, 0);
    lv_line_set_points(line, app->tick_points[index], 2);
}

static void create_numeral(lv_obj_t *parent, lv_obj_t **label, const char *text, float angle_deg, int x_adjust, int y_adjust)
{
    const float angle = (angle_deg - 90.0f) * (APP_PI / 180.0f);
    const int radius = 54;
    const int x = APP_DIAL_RADIUS + (int)lroundf(cosf(angle) * radius) + x_adjust;
    const int y = APP_DIAL_RADIUS + (int)lroundf(sinf(angle) * radius) + y_adjust;

    *label = lv_label_create(parent);
    lv_label_set_text(*label, text);
    lv_obj_set_style_text_color(*label, lv_color_hex(0xF2F4F7), 0);
    lv_obj_set_style_text_opa(*label, LV_OPA_90, 0);
    lv_obj_set_style_text_letter_space(*label, 1, 0);
    lv_obj_set_pos(*label, x, y);
}

static void hand_rotation_exec_cb(void *var, int32_t value)
{
    hand_widget_t *hand = (hand_widget_t *)var;
    int32_t rotation = normalize_rotation_tenths(value);
    float angle = ((float)rotation / 10.0f - 90.0f) * (APP_PI / 180.0f);
    float dir_x = cosf(angle);
    float dir_y = sinf(angle);

    hand->points[0].x = APP_DIAL_RADIUS - dir_x * hand->tail_len;
    hand->points[0].y = APP_DIAL_RADIUS - dir_y * hand->tail_len;
    hand->points[1].x = APP_DIAL_RADIUS + dir_x * hand->head_len;
    hand->points[1].y = APP_DIAL_RADIUS + dir_y * hand->head_len;
    hand->current_rotation = rotation;

    lv_obj_invalidate(hand->line);
    if (hand->counterweight != NULL) {
        lv_obj_set_pos(hand->counterweight,
                       (int)lroundf(APP_DIAL_RADIUS - dir_x * hand->tail_len) - hand->counterweight_size / 2,
                       (int)lroundf(APP_DIAL_RADIUS - dir_y * hand->tail_len) - hand->counterweight_size / 2);
    }
}

static void hand_widget_create(hand_widget_t *hand, lv_obj_t *parent, int32_t head_len, int32_t tail_len,
                               int32_t line_width, lv_color_t color, uint8_t opacity, int32_t counterweight_size)
{
    memset(hand, 0, sizeof(*hand));
    hand->head_len = head_len;
    hand->tail_len = tail_len;
    hand->counterweight_size = counterweight_size;

    hand->line = lv_line_create(parent);
    lv_obj_set_size(hand->line, APP_DIAL_SIZE, APP_DIAL_SIZE);
    lv_obj_set_pos(hand->line, 0, 0);
    lv_obj_set_style_bg_opa(hand->line, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(hand->line, 0, 0);
    lv_obj_set_style_pad_all(hand->line, 0, 0);
    lv_obj_set_style_line_width(hand->line, line_width, 0);
    lv_obj_set_style_line_color(hand->line, color, 0);
    lv_obj_set_style_line_opa(hand->line, opacity, 0);
    lv_obj_set_style_line_rounded(hand->line, true, 0);
    lv_line_set_points_mutable(hand->line, hand->points, 2);

    if (counterweight_size > 0) {
        hand->counterweight = lv_obj_create(parent);
        lv_obj_set_size(hand->counterweight, counterweight_size, counterweight_size);
        lv_obj_set_style_radius(hand->counterweight, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(hand->counterweight, color, 0);
        lv_obj_set_style_bg_opa(hand->counterweight, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(hand->counterweight, 0, 0);
        lv_obj_set_style_outline_width(hand->counterweight, 1, 0);
        lv_obj_set_style_outline_color(hand->counterweight, lv_color_hex(0x2A0A08), 0);
        lv_obj_set_style_outline_opa(hand->counterweight, LV_OPA_60, 0);
    }
}

static void clock_apply_hand_pose(hand_widget_t *hand, int32_t target)
{
    lv_anim_delete(hand, hand_rotation_exec_cb);
    hand_rotation_exec_cb(hand, target);
}

static void clock_animate_hand(hand_widget_t *hand, int32_t target, uint32_t duration_ms)
{
    int32_t normalized_target = normalize_rotation_tenths(target);
    int32_t delta = forward_rotation_delta(hand->current_rotation, normalized_target);
    int32_t start_value = hand->current_rotation;
    int32_t end_value = hand->current_rotation + delta;

    lv_anim_delete(hand, hand_rotation_exec_cb);

    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, hand);
    lv_anim_set_exec_cb(&anim, hand_rotation_exec_cb);
    lv_anim_set_values(&anim, start_value, end_value);
    lv_anim_set_duration(&anim, duration_ms);
    lv_anim_set_path_cb(&anim, lv_anim_path_linear);
    lv_anim_set_early_apply(&anim, true);
    lv_anim_start(&anim);

    hand->current_rotation = normalized_target;
}

static void clock_update_info_label(app_state_t *app)
{
    char text[32];
    clock_model_format(&app->clock, text, sizeof(text));
    lv_label_set_text(app->info_label, text);
}

static void clock_tick_timer_cb(lv_timer_t *timer)
{
    app_state_t *app = (app_state_t *)lv_timer_get_user_data(timer);
    int32_t hour_angle = 0;
    int32_t minute_angle = 0;
    int32_t second_angle = 0;
    clock_model_get_angles(&app->clock, &hour_angle, &minute_angle, &second_angle);

    clock_animate_hand(&app->hour_hand, hour_angle, APP_HAND_ANIM_MS);
    clock_animate_hand(&app->minute_hand, minute_angle, APP_HAND_ANIM_MS);
    clock_animate_hand(&app->second_hand, second_angle, APP_HAND_ANIM_MS);
    clock_update_info_label(app);
}

static void clock_start(app_state_t *app)
{
    int32_t hour_angle = 0;
    int32_t minute_angle = 0;
    int32_t second_angle = 0;
    clock_model_get_angles(&app->clock, &hour_angle, &minute_angle, &second_angle);

    clock_apply_hand_pose(&app->hour_hand, hour_angle);
    clock_apply_hand_pose(&app->minute_hand, minute_angle);
    clock_apply_hand_pose(&app->second_hand, second_angle);
    clock_update_info_label(app);

    app->clock_timer = lv_timer_create(clock_tick_timer_cb, 1000, app);
}

static void ui_create(app_state_t *app)
{
    lv_obj_t *screen = lv_display_get_screen_active(app->display);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    app->face = lv_obj_create(screen);
    lv_obj_set_size(app->face, APP_DIAL_SIZE, APP_DIAL_SIZE);
    lv_obj_center(app->face);
    lv_obj_set_style_radius(app->face, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(app->face, lv_color_hex(0x050607), 0);
    lv_obj_set_style_bg_opa(app->face, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(app->face, 2, 0);
    lv_obj_set_style_border_color(app->face, lv_color_hex(0x2C3138), 0);
    lv_obj_set_style_outline_width(app->face, 1, 0);
    lv_obj_set_style_outline_color(app->face, lv_color_hex(0x101418), 0);
    lv_obj_set_style_outline_opa(app->face, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(app->face, 0, 0);
    lv_obj_clear_flag(app->face, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *inner_ring = lv_obj_create(app->face);
    lv_obj_set_size(inner_ring, 176, 176);
    lv_obj_center(inner_ring);
    lv_obj_set_style_radius(inner_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(inner_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(inner_ring, 1, 0);
    lv_obj_set_style_border_color(inner_ring, lv_color_hex(0x171B20), 0);
    lv_obj_set_style_outline_width(inner_ring, 0, 0);
    lv_obj_set_style_pad_all(inner_ring, 0, 0);

    for (uint32_t i = 0; i < APP_TICK_COUNT; i++) {
        create_tick(app, app->face, i);
    }

    create_numeral(app->face, &app->numerals[0], "12", 0.0f, -9, -11);
    create_numeral(app->face, &app->numerals[1], "3", 90.0f, -4, -8);
    create_numeral(app->face, &app->numerals[2], "6", 180.0f, -4, -6);
    create_numeral(app->face, &app->numerals[3], "9", 270.0f, -4, -8);

    hand_widget_create(&app->hour_hand, app->face, 52, 0, 6, lv_color_hex(0xB9C0C8), LV_OPA_COVER, 0);
    hand_widget_create(&app->minute_hand, app->face, 74, 0, 4, lv_color_hex(0xF4F7FA), LV_OPA_COVER, 0);
    hand_widget_create(&app->second_hand, app->face, 86, 24, 2, lv_color_hex(0xFF453A), LV_OPA_COVER, 8);

    app->center_outer = lv_obj_create(app->face);
    lv_obj_set_size(app->center_outer, 14, 14);
    lv_obj_center(app->center_outer);
    lv_obj_set_style_radius(app->center_outer, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(app->center_outer, lv_color_hex(0xC8CDD4), 0);
    lv_obj_set_style_bg_opa(app->center_outer, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(app->center_outer, 0, 0);

    app->center_inner = lv_obj_create(app->face);
    lv_obj_set_size(app->center_inner, 6, 6);
    lv_obj_center(app->center_inner);
    lv_obj_set_style_radius(app->center_inner, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(app->center_inner, lv_color_hex(0x101214), 0);
    lv_obj_set_style_bg_opa(app->center_inner, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(app->center_inner, 0, 0);

    app->info_label = lv_label_create(screen);
    lv_label_set_text(app->info_label, "SIM  10:08:36");
    lv_obj_align(app->info_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_color(app->info_label, lv_color_hex(0x5E6670), 0);
    lv_obj_set_style_text_opa(app->info_label, LV_OPA_80, 0);
    lv_obj_set_style_text_letter_space(app->info_label, 2, 0);
}

void app_main(void)
{
    app_state_t *app = &s_app;
    memset(app, 0, sizeof(*app));

    clock_model_init(&app->clock);
    lcd_display_init(app);
    lvgl_init_display(app);

    _lock_acquire(&s_lvgl_lock);
    ui_create(app);
    clock_start(app);
    _lock_release(&s_lvgl_lock);

    xTaskCreate(lvgl_port_task, "lvgl", APP_LVGL_TASK_STACK_SIZE, NULL, APP_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Minimal analog clock ready (%s time source)", app->clock.use_system_time ? "system" : "simulated");
}
