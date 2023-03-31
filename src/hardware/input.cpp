#include "hardware/input.hpp"
#include "hardware/hal.hpp"
#include "tools.hpp"
#include "ui/ui_main.hpp"
#include "watch/watch.hpp"

constexpr auto LONG_PRESS = SEC_IN_MS * 1; // 1 sec

void input_manager::setup()
{
    DEBUG_PRINT("input_manager setup : ");
    pinMode(TP_PIN, GPIO_MODE_INPUT);
    pinMode(TP_PIN_POWER, PULLUP);
    digitalWrite(TP_PIN_POWER, HIGH);
    _to_input.reset_delay(TIME_OUT_INPUT_TO_SLEEP_MS);
    DEBUG_PRINTLN("done");
}

void input_manager::update()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    const bool current_state = (digitalRead(TP_PIN) == HIGH);

    /* press */
    if (!_last_state && current_state)
    {
        _presse_time = millis();
    } else
    {
        /* release */
        if (_last_state && !current_state)
        {
            _release_time = millis();

            if ((_release_time - _presse_time) >= LONG_PRESS)
            {
                _is_long_press = true;
            }
            _action_read = false;
            /* Compass page -1 */
            _to_input.reset_delay(_watch->ui_m.get_page() == ui_page::weather ? TIME_OUT_INPUT_TO_LONG_SLEEP_MS
                                                                              : TIME_OUT_INPUT_TO_SLEEP_MS);
            _counter_long = 0;
        } else
        {
            if (_last_state && current_state && (millis() - _presse_time) >= LONG_PRESS)
            {
                _is_long_press = true;
                _action_read = false;
                _counter_long = _counter_long < 10 ? _counter_long + 1 : _counter_long;
            }
        }
    }
    _last_state = current_state;

    xSemaphoreGive(_mutex);
}

auto input_manager::get_pressed_type() -> input_type
{
    input_type ret = input_type::NONE;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    if (!_action_read && _counter_long <= 1)
    {
        _action_read = true;
        _counter_long = 5;
        if (_is_long_press)
        {
            _is_long_press = false;
            ret = input_type::LONG_PRESS;
        } else
        {
            ret = input_type::PRESS;
        }
    }
    xSemaphoreGive(_mutex);
    return ret;
}

auto input_manager::is_time_out() -> bool
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    const bool ret = _to_input.is_time_out();
    xSemaphoreGive(_mutex);
    return ret;
}

void input_manager::reset_to()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _to_input.reset_delay(_watch->ui_m.get_page() == ui_page::compass ? TIME_OUT_INPUT_TO_LONG_SLEEP_MS
                                                                      : TIME_OUT_INPUT_TO_SLEEP_MS);
    xSemaphoreGive(_mutex);
}
