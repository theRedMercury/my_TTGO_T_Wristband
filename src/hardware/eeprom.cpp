

#include "hardware/eeprom.hpp"
#include "hardware/hal.hpp"
#include "tools.hpp"

#include <EEPROM.h>

void epprom_mem::store_wifi_mem(uint8_t value)
{
    _store_mem(WIFI_STATE_ADDRESS, value);
}

auto epprom_mem::get_wifi_mem() -> uint8_t
{
    return _get_mem(WIFI_STATE_ADDRESS);
}

void epprom_mem::_store_mem(uint16_t address, uint8_t value)
{
    EEPROM.begin(1);
    EEPROM.put(address, value);
    DEBUG_PRINT("Save : ");
    DEBUG_PRINTLN(value);
    EEPROM.commit();
    EEPROM.end();
}

auto epprom_mem::_get_mem(uint16_t address) -> uint8_t
{
    EEPROM.begin(1);
    const uint8_t value = EEPROM.read(address);
    DEBUG_PRINT("Get : ");
    DEBUG_PRINTLN(value);
    EEPROM.end();
    return value;
}
