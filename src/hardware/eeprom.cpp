

#include "hardware/eeprom.hpp"
#include "hardware/hal.hpp"
#include "tools.hpp"

SemaphoreHandle_t epprom_mem::_mem_lock = xSemaphoreCreateMutex();

void epprom_mem::store_wifi_mem(uint8_t value)
{
    xSemaphoreTake(_mem_lock, portMAX_DELAY);
    EEPROM_WRITE(WIFI_STATE_ADDRESS, value);
    xSemaphoreGive(_mem_lock);
}

auto epprom_mem::get_wifi_mem() -> uint8_t
{
    uint8_t v = 0;
    xSemaphoreTake(_mem_lock, portMAX_DELAY);
    EEPROM_READ(WIFI_STATE_ADDRESS, v);
    xSemaphoreGive(_mem_lock);
    return v;
}

void epprom_mem::store_steps_mem(ulong value)
{
    xSemaphoreTake(_mem_lock, portMAX_DELAY);
    EEPROM_WRITE(STEPS_ADDRESS, value);
    xSemaphoreGive(_mem_lock);
}

auto epprom_mem::get_steps_mem() -> ulong
{
    ulong v = 0;
    xSemaphoreTake(_mem_lock, portMAX_DELAY);
    EEPROM_READ(STEPS_ADDRESS, v);
    xSemaphoreGive(_mem_lock);
    return v;
}

void epprom_mem::store_step_time_mem(ulong value)
{
    xSemaphoreTake(_mem_lock, portMAX_DELAY);
    EEPROM_WRITE(STEPS_TIME_ADDRESS, value);
    xSemaphoreGive(_mem_lock);
}

auto epprom_mem::get_steps_time_mem() -> ulong
{
    ulong v = 0;
    xSemaphoreTake(_mem_lock, portMAX_DELAY);
    EEPROM_READ(STEPS_TIME_ADDRESS, v);
    xSemaphoreGive(_mem_lock);
    return v;
}
