#pragma once

#include <Arduino.h>
#include <EEPROM.h>

class epprom_mem final
{
public:
    static void store_wifi_mem(uint8_t value);
    static auto get_wifi_mem() -> uint8_t;

    static void store_steps_mem(ulong value);
    static auto get_steps_mem() -> ulong;

    static void store_step_time_mem(ulong value);
    static auto get_steps_time_mem() -> ulong;

private:
    template <class T>
    static auto EEPROM_WRITE(int address_start, const T& value) -> size_t
    {
        const uint8_t* p = (const uint8_t*)(const void*)&value;

        size_t i;
        EEPROM.begin(sizeof(value));
        for (i = 0; i < sizeof(value); i++)
        {
            EEPROM.write(address_start++, *p++);
        }
        EEPROM.commit();
        EEPROM.end();
        return i;
    }

    template <class T>
    static auto EEPROM_READ(int address_start, T& value) -> size_t
    {
        uint8_t* p = (uint8_t*)(void*)&value;
        size_t i;
        EEPROM.begin(sizeof(value));
        for (i = 0; i < sizeof(value); i++)
        {
            *p++ = EEPROM.read(address_start++);
        }
        EEPROM.end();
        return i;
    }

    static SemaphoreHandle_t _mem_lock;
};
