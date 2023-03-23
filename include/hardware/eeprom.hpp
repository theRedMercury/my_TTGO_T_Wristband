#pragma once

#include <Arduino.h>

class epprom_mem final
{
public:
    static void store_wifi_mem(uint8_t value);
    static auto get_wifi_mem() -> uint8_t;

private:
    static void _store_mem(uint16_t address, uint8_t value);
    static auto _get_mem(uint16_t address) -> uint8_t;
};
