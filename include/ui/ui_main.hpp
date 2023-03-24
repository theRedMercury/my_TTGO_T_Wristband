#pragma once
#include <Arduino.h>

#include "hardware/hal.hpp"
#include "tools.hpp"
#include "watch/watch_abs.hpp"

enum class ui_page : int8_t
{
    charging = 0,
    main,
    weather,
    compass,
    running,
    chrono,
    mpu,
};

class main_ui final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup() override;

    void next_page();
    auto get_page() const -> ui_page;

    void set_charging();
    void handle_ui();

    auto get_need_redraw() const -> bool;

    void update_sec(bool d);
    void update_wifi(uint8_t counter);

private:
    void _draw_main_page();
    void _draw_weather_page();
    void _draw_compass_page();
    void _draw_running_page();
    void _draw_chrono_page();
    void _draw_mpu_page();
    void _draw_charging_page();

    ui_page _page = ui_page::main;
    bool _need_redraw = false;
    SemaphoreHandle_t _lock_screen = xSemaphoreCreateMutex();
};
