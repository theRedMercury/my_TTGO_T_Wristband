#pragma once

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "watch/watch_abs.hpp"

/**
 * @brief NEED file : _personal_conf.hpp
 *
 * // WIFI Settings
struct wifi_conf
{
    const char* ssid;
    const char* pwd;
};

constexpr wifi_conf wifi_config[] = {
    {"SSID1", "111"},
    {"SSID2", "222"},
};
 */
enum class wifi_signal : uint8_t
{
    S_OFF = 0,
    S_ON,
    S_CONNECTED,
    S_LOW,
    S_FAIR,
    S_GOOD,
    S_EXCEL,
};

class wifi_manager final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup() override;
    void update();
    // static void config_mode_callback(WiFiManager* wifi_m);
    void activate();
    void deactivate(bool save = false);

    auto is_state_change() const -> bool;
    auto is_connected() const -> bool;
    auto is_on() const -> bool;
    /**
     * @brief Get the strenght object
     *
     * @return wifi_signal
     */
    auto get_strenght() const -> wifi_signal;

private:
    auto _connect() -> bool;
    bool _state_change = false;
    bool _need_update = false;
    wifi_signal _signal = wifi_signal::S_OFF;
    SemaphoreHandle_t _lock = xSemaphoreCreateMutex();
};
