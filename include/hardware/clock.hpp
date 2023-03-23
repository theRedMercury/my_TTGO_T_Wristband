#pragma once

#include <Arduino.h>
#include <pcf8563.h>

#include "tools.hpp"
#include "watch/watch_abs.hpp"

class clock_manager final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup();
    void deep_sleep();
    void set_time(RTC_Date datetime);

    auto get_clock_time() -> RTC_Date;
    auto get_utc_time() -> RTC_Date;
    auto is_dst(RTC_Date now) -> bool;

    void start_chrono();
    void stop_chrono();
    void reset_chrono();
    void reset_to();
    auto get_chrono_time() const -> ulong;
    auto get_chrono_running() const -> bool;
    auto get_chrono_start_ready() -> bool;

private:
    PCF8563_Class _rtc;

    bool _chrono_running = false;
    ulong _chrono_time_s = 0;
    ulong _chrono_time_stop = 0;
    delay_time_out _running_to{3000};

    SemaphoreHandle_t _lock = xSemaphoreCreateMutex();
};
