#include "wifi/wifi.hpp"
#include "hardware/eeprom.hpp"
#include "tools.hpp"
#include "wifi/_personal_conf.hpp"

void wifi_manager::setup()
{
    DEBUG_PRINT("wifi_manager setup : ");
    _signal = static_cast<wifi_signal>(epprom_mem::get_wifi_mem());
    _state_change = true;
    _need_update = true;
    DEBUG_PRINTLN("done");
}

void wifi_manager::update()
{
    if (_state_change || _need_update)
    {
        _state_change = false;
        _need_update = false;

        if (_signal == wifi_signal::S_ON)
        {
            if (!_connect())
            {
                _state_change = false;
                _need_update = false;
            }
        }
        if (_signal == wifi_signal::S_OFF)
        {
            WiFi.mode(WIFI_OFF);
            return;
        }
    }

    const wifi_signal old_signal = _signal;
    if (WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED)
    {
        return;
    }
    const int8_t _strenght = WiFi.RSSI();

    if (_strenght > -50)
    {
        _signal = wifi_signal::S_EXCEL;
    } else
    {
        if (_strenght > -60)
        {
            _signal = wifi_signal::S_GOOD;
        } else
        {
            if (_strenght > -70)
            {
                _signal = wifi_signal::S_FAIR;
            } else
            {
                _signal = wifi_signal::S_LOW;
            }
        }
    }

    _state_change = old_signal != _signal;
}

auto wifi_manager::_connect() -> bool
{
    if (_signal == wifi_signal::S_OFF)
    {
        return false;
    }

    for (const auto& wifi_c : wifi_config)
    {
        DEBUG_PRINTLN("Try : " + String(wifi_c.ssid));
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifi_c.ssid, wifi_c.pwd);
        for (uint8_t i = 0; i < 22; ++i)
        {
            DEBUG_PRINT('.');
            if (WiFi.status() == WL_CONNECTED)
            {
                _need_update = true;
                DEBUG_PRINTLN("connected");
                return true;
            }
            delay(100);
        }
        WiFi.mode(WIFI_OFF);
        delay(100);
    }
    _state_change = true;
    DEBUG_PRINTLN("no wifi found");
    return false;
}

auto wifi_manager::is_state_change() const -> bool
{
    return _state_change;
}

auto wifi_manager::is_connected() const -> bool
{
    return _signal >= wifi_signal::S_CONNECTED;
}

auto wifi_manager::is_on() const -> bool
{
    return _signal >= wifi_signal::S_ON;
}

auto wifi_manager::get_strenght() const -> wifi_signal
{
    return _signal;
}

void wifi_manager::activate()
{
    DEBUG_PRINTLN("activate");
    _signal = wifi_signal::S_ON;
    epprom_mem::store_wifi_mem(static_cast<uint8_t>(_signal));
    _need_update = true;
}

void wifi_manager::deactivate(bool save)
{
    DEBUG_PRINTLN("deactivate");
    _signal = wifi_signal::S_OFF;
    if (save)
    {
        epprom_mem::store_wifi_mem(static_cast<uint8_t>(_signal));
    }
    _need_update = true;
}
