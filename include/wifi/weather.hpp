#pragma once

#include <Arduino.h>

class weather_api final
{
public:
    static auto update() -> bool;
    static auto get_degs_str(int16_t cap) -> char const*;

    static char current_str[18];
    static float temp;
    static float temp_min;
    static float temp_max;
    static float temp_feel;
    static uint16_t pressure;
    static uint8_t humidity;

    static float wind_speed;
    static int16_t wind_degs;
    static uint8_t clouds; // 0 - 100
};
