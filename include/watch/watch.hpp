#pragma once

#include "hardware/battery.hpp"
#include "hardware/clock.hpp"
#include "hardware/hal.hpp"
#include "hardware/input.hpp"
#include "hardware/mpu.hpp"
#include "tools.hpp"
#include "ui/tft.hpp"
#include "ui/ui_main.hpp"
#include "wifi/ntp.hpp"
#include "wifi/weather.hpp"
#include "wifi/wifi.hpp"

class watch_manager final
{

public:
    watch_manager()
    {
    }
    watch_manager(const watch_manager&) = delete;
    watch_manager(watch_manager&&) = delete;
    watch_manager& operator=(const watch_manager&) = delete;
    watch_manager& operator=(watch_manager&&) = delete;

    void setup();

    auto go_to_sleep() -> bool;
    void normal_mode();

    void modem_sleep(bool change_freq = true);
    void light_sleep();
    void deep_sleep();

    auto get_need_update_ui() const -> bool;
    void set_need_update_ui(bool update);

    void update_step();

    static void update_gui(void* param);
    static void update_gui_sec(void* param);
    static void update_bat_state(void* param);
    static void update_wifi_pipeline(void* param);
    static void input_trigger(void* param);
    static void update_mpu(void* param);

    clock_manager clk_m{*this};
    battery_manager bat_m{*this};
    mpu_manager mpu_m{*this};
    wifi_manager wifi_m{*this};
    tft_screen screen{*this};
    ntp_manager ntp_m{*this};
    main_ui ui_m{*this};
    input_manager input_m{*this};

    SemaphoreHandle_t i2c_lock = xSemaphoreCreateMutex();

    inline auto is_in_modem_sleep() const -> bool
    {
        return _is_in_modem_sleep;
    }

    void reset_mpu_step();

private:
    auto _time_to_light_sleep() -> bool;

    TaskHandle_t _handle_update_gui = NULL;
    TaskHandle_t _handle_update_gui_sec = NULL;
    TaskHandle_t _handle_update_mpu = NULL;
    TaskHandle_t _handle_update_bat_state = NULL;
    TaskHandle_t _handle_update_wifi_pipeline = NULL;

    bool _is_in_modem_sleep = false;
    bool _need_update_ui = true;
};
