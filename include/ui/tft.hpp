#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include "hardware/hal.hpp"
#include "watch/watch_abs.hpp"

#include "ubuntu_light_52.hpp"
// #include "texgyrecursor_regular_52.hpp"

constexpr auto MAIN_CLOCK_FONT = ubuntu_light_52;

#define ST7735_IDMOFF 0x38
#define ST7735_IDMON 0x39
#define ST7735_COLMOD 0x3A

class tft_screen final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup() override;
    void set_backlight(bool on = true) const;
    void clear();
    void idle();
    void deep_sleep();
    void wake_up();

    void play_anim_transition();
    void load_font(const uint8_t* array, bool load = true);

    void show_main_page(const uint8_t hour, const uint8_t minute, const uint8_t day, const uint8_t month, bool utc);
    void show_battery_page(float voltage, uint8_t percentage, bool charging);
    void show_weather_page();
    void show_running_page();
    void show_compass_page();
    void show_chrono_page();
    void show_mpu_page();

    uint16_t display_sec_dot(bool color, bool utc);

    void draw_wifi_signal(uint8_t counter);

private:
    void _show_header(const uint8_t day, const uint8_t month);
    void _draw_battery_arc();
    void _draw_wifi_arc(const uint8_t counter);
    void _draw_date(const uint8_t day, const uint8_t month);
    void _draw_clock(const uint8_t hour, const uint8_t minute);
    void _draw_basic_wheater();

    static TFT_eSPI _tft;

    int16_t _percent_bat = 0;
    uint8_t _h, _m, _d, _mo = 0;
    uint8_t _sec_dot_xpos = 0;
    uint8_t _y_time_pos = 0;

    // {x, y, z}
    const int16_t _orig_points[4][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const int16_t _orig_points_c[5][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}};

    const float z_offset = -2.0;
    const float cube_size = 60.0;

    bool _is_in_idle = false;
};
