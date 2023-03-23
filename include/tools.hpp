#pragma once

#include <Arduino.h>
#include <HTTPClient.h>

// #define DEBUG_MODE
////////////////////////////////////////////////////////
#ifdef DEBUG_MODE
#define DEBUG_PRINTER Serial
#define DEBUG_PRINT(...)                                                                                               \
    {                                                                                                                  \
        DEBUG_PRINTER.print(__VA_ARGS__);                                                                              \
    }
#define DEBUG_PRINTLN(...)                                                                                             \
    {                                                                                                                  \
        DEBUG_PRINTER.println(__VA_ARGS__);                                                                            \
    }
#else
#define DEBUG_PRINT(...)                                                                                               \
    {                                                                                                                  \
    }
#define DEBUG_PRINTLN(...)                                                                                             \
    {                                                                                                                  \
    }
#endif
////////////////////////////////////////////////////////

template <typename T>
constexpr void SAFE_DELETE_TASK(T t)
{
    if (t)
    {
        vTaskDelete(t);
    }
}

class tools final
{
public:
    template <typename T>
    static auto clamp(T value, T min, T max) -> T
    {
        return (value < min) ? min : (value > max) ? max : value;
    }

    template <typename T>
    static auto changed(T value_new, T value_old, T threshold) -> T
    {
        return (value_new > (value_old + threshold)) || (value_new < (value_old - threshold));
    }

    template <typename T>
    static auto normalize(T value, T min, T max) -> T
    {
        return (value - min) / (max - min);
    }

    template <typename T, typename U>
    static auto map(T x, T in_min, T in_max, U out_min, U out_max) -> U
    {
        return static_cast<U>((x - in_min) * static_cast<T>(out_max - out_min) / (in_max - in_min) +
                              static_cast<T>(out_min));
    }

    static auto split_str(const String data, const unsigned int index, const char separator) -> String
    {
        unsigned int found = 0;
        int str_index[] = {0, -1};
        const int max_index = data.length() - 1;
        for (int i = 0; i <= max_index && found <= index; ++i)
        {
            if (data.charAt(i) == separator || i == max_index)
            {
                found++;
                str_index[0] = str_index[1] + 1;
                str_index[1] = (i == max_index) ? i + 1 : i;
            }
        }
        return found > index ? data.substring(str_index[0], str_index[1]) : "";
    }

    static auto http_get_request(const char* server_name) -> String
    {
        HTTPClient http;
        http.begin(server_name);

        const int httpResponseCode = http.GET();

        String payload = "{}";

        if (httpResponseCode > 0)
        {
            DEBUG_PRINT("HTTP Response code: ");
            DEBUG_PRINTLN(httpResponseCode);
            payload = http.getString();
        } else
        {
            DEBUG_PRINT("Error code: ");
            DEBUG_PRINTLN(httpResponseCode);
        }
        http.end();

        return payload;
    }
};

class delay_time_out final
{
public:
    delay_time_out(const unsigned long ms_delay = 250)
    {
        _ms_delay = ms_delay;
    }

    void reset_delay(const unsigned long ms_delay_overwrite = 0)
    {
        if (ms_delay_overwrite != 0)
        {
            _next_millis = millis() + ms_delay_overwrite;
        } else
        {
            _next_millis = millis() + _ms_delay;
        }
    }

    auto is_time_out(const bool reset = false, const unsigned long add_delay = 0) -> bool
    {
        const bool is_timeout = (_next_millis + add_delay) <= millis();
        if (is_timeout)
        {
            if (reset)
            {
                reset_delay();
                if (_next_millis + add_delay <= millis())
                {
                    if (!_overload)
                    {
                        _overload = true;
                        return true;
                    }
                    return false;
                }
            } else
            {
                // Keep in timeout
                _next_millis = 0;
            }
        }

        _overload = false;
        return is_timeout;
    }

    delay_time_out() = delete;
    delay_time_out(const delay_time_out&) = delete;
    delay_time_out(delay_time_out&&) = delete;
    delay_time_out& operator=(const delay_time_out&) = delete;
    delay_time_out& operator=(delay_time_out&&) = delete;

private:
    unsigned long _next_millis = 0;
    unsigned long _ms_delay;
    bool _overload = false;
};
