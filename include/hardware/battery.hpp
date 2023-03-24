#pragma once

#include <Arduino.h>
#include <esp_adc_cal.h>

#include "watch/watch_abs.hpp"

class battery_manager final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup() override;
    void update();
    void update_light();

    auto is_charging() const -> bool;
    auto state_changed() const -> bool;
    auto get_voltage() const -> float;
    auto get_percentage() const -> uint8_t;

private:
    float _light = 0.F;
    bool _state_change = false;
    bool _is_charging = false;
};
