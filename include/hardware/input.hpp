#pragma once

#include "tools.hpp"
#include "watch/watch_abs.hpp"

constexpr auto TIME_OUT_INPUT_TO_SLEEP_MS = SEC_IN_MS * 15; // 15 sec

enum class input_type : uint8_t
{
    NONE = 0,
    PRESS,
    LONG_PRESS,
};

class input_manager final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup() override;
    void update();
    auto get_pressed_type() -> input_type;

    auto is_time_out() -> bool;
    void reset_to();

private:
    bool _last_state = false;
    ulong _presse_time = 0;
    ulong _release_time = 0;
    bool _action_read = true;
    uint8_t _counter_long = 0;
    bool _is_long_press = false;
    SemaphoreHandle_t _mutex = xSemaphoreCreateMutex();

    delay_time_out _to_input{TIME_OUT_INPUT_TO_SLEEP_MS};
};
