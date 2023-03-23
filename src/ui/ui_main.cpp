#include "ui/ui_main.hpp"
#include "tools.hpp"
#include "watch/watch.hpp"

void main_ui::setup()
{
    _draw_main_page();
}

auto main_ui::get_need_redraw() const -> bool
{
    return _need_redraw;
}

void main_ui::next_page()
{
    if (xSemaphoreTake(_lock_screen, portMAX_DELAY))
    {
        _watch->screen.play_anim_transition();
        _page = static_cast<ui_page>(static_cast<int>(_page) + 1);
        _watch->set_need_update_ui(true);
        xSemaphoreGive(_lock_screen);
    }
}

auto main_ui::get_page() const -> ui_page
{
    return _page;
}

void main_ui::set_charging()
{
    if (xSemaphoreTake(_lock_screen, portMAX_DELAY))
    {
        _watch->screen.clear();
        _page = ui_page::charging;
        xSemaphoreGive(_lock_screen);
    }
}

void main_ui::handle_ui()
{
    if (xSemaphoreTake(_lock_screen, portMAX_DELAY))
    {
        switch (_page)
        {
        case ui_page::charging:
            _draw_charging_page();
            break;
        case ui_page::main:
            _draw_main_page();
            break;
        case ui_page::weather:
            _draw_weather_page();
            break;
        case ui_page::compass:
            _draw_compass_page();
            break;
        case ui_page::running:
            _draw_running_page();
            break;
        case ui_page::chrono:
            _draw_chrono_page();
            break;
        case ui_page::mpu:
            _draw_mpu_page();
            break;
        default:
            _page = ui_page::main;
            _draw_main_page();
            break;
        }
        xSemaphoreGive(_lock_screen);
    }
}

void main_ui::update_sec(bool d)
{
    if (xSemaphoreTake(_lock_screen, portMAX_DELAY))
    {
        if (_page == ui_page::main)
        {
            _watch->screen.load_font(MAIN_CLOCK_FONT, true);
            _watch->screen.display_sec_dot(d, true);
            _watch->screen.load_font(nullptr, false);
        }
        xSemaphoreGive(_lock_screen);
    }
}

void main_ui::update_wifi(uint8_t counter)
{
    if (xSemaphoreTake(_lock_screen, portMAX_DELAY))
    {
        if (_page == ui_page::main)
        {
            _watch->screen.draw_wifi_signal(counter);
        }
        xSemaphoreGive(_lock_screen);
    }
}

void main_ui::_draw_charging_page()
{
    _watch->screen.show_battery_page(_watch->bat_m.get_voltage(), _watch->bat_m.get_percentage(), true);
    _need_redraw = true;
}

void main_ui::_draw_main_page()
{
    const RTC_Date d = _watch->clk_m.get_clock_time();
    _watch->screen.show_main_page(d.hour, d.minute, d.day, d.month, true);
    _need_redraw = false;
}

void main_ui::_draw_weather_page()
{
    _watch->screen.show_weather_page();
    _need_redraw = false;
}

void main_ui::_draw_compass_page()
{
    _watch->screen.show_compass_page();
    _need_redraw = true;
}

void main_ui::_draw_running_page()
{
    _watch->screen.show_running_page();
    _need_redraw = false;
}

void main_ui::_draw_chrono_page()
{
    _watch->screen.show_chrono_page();
    _need_redraw = true;
}

void main_ui::_draw_mpu_page()
{
    _watch->screen.show_mpu_page();
    _need_redraw = true;
}
