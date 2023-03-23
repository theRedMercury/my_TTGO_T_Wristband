#include "hardware/clock.hpp"
#include "hardware/hal.hpp"
#include "watch/watch.hpp"

RTC_DATA_ATTR uint8_t current_day = 0;

void clock_manager::setup()
{
    DEBUG_PRINT("clock_manager setup : ");
    _rtc.begin(Wire);
    _rtc.check();
    DEBUG_PRINTLN("done");
}

void clock_manager::deep_sleep()
{
    _rtc.clearTimer();
    _rtc.resetAlarm();
    _rtc.disableAlarm();
    _rtc.disableCLK();
    _rtc.disableTimer();
}

void clock_manager::set_time(RTC_Date datetime)
{
    const uint8_t c_day = current_day;
    _rtc.setDateTime(datetime);
    current_day = datetime.day;

    /* New Day */
    if (c_day != current_day)
    {
        _watch->mpu_m.reset_step();
        _watch->set_need_update_ui(true);
    }
}

auto clock_manager::get_clock_time() -> RTC_Date
{
    return _rtc.getDateTime();
}

auto clock_manager::get_utc_time() -> RTC_Date
{
    const RTC_Date now = _rtc.getDateTime();
    tm timeStructure;
    timeStructure.tm_hour = now.hour;
    timeStructure.tm_min = now.minute;
    timeStructure.tm_sec = now.second;
    timeStructure.tm_mday = now.day;
    timeStructure.tm_mon = now.month - 1;
    timeStructure.tm_year = now.year - 1900;
    timeStructure.tm_isdst = -1;

    timeStructure.tm_hour = is_dst(now) ? timeStructure.tm_hour - 2 : timeStructure.tm_hour - 1;

    time_t gmtTime = mktime(&timeStructure);
    const tm* gmtStructure = localtime(&gmtTime);
    return RTC_Date(gmtStructure->tm_year + 1900, gmtStructure->tm_mon + 1, gmtStructure->tm_mday,
                    gmtStructure->tm_hour, gmtStructure->tm_min, gmtStructure->tm_sec);
}

auto clock_manager::is_dst(RTC_Date now) -> bool
{
    const uint8_t dayOfWeek = _rtc.getDayOfWeek(now.day, now.month, now.year);
    if (now.month < 3 || now.month > 10)
    {
        return false;
    }
    if (now.month > 3 && now.month < 10)
    {
        return true;
    }
    return (now.day - dayOfWeek) >= 20;
}

void clock_manager::start_chrono()
{
    xSemaphoreTake(_lock, portMAX_DELAY);
    _chrono_time_s = millis();
    _chrono_running = true;
    xSemaphoreGive(_lock);
}

void clock_manager::stop_chrono()
{
    xSemaphoreTake(_lock, portMAX_DELAY);
    _chrono_time_stop = millis();
    _chrono_running = false;
    _running_to.reset_delay(1000 * 60);
    xSemaphoreGive(_lock);
}

void clock_manager::reset_chrono()
{
    xSemaphoreTake(_lock, portMAX_DELAY);
    _chrono_time_s = 0;
    _chrono_time_stop = 0;
    _chrono_running = false;
    _running_to.reset_delay(3000);
    xSemaphoreGive(_lock);
}

void clock_manager::reset_to()
{
    _running_to.reset_delay(3000);
}

auto clock_manager::get_chrono_time() const -> ulong
{
    ulong t = 0;
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (_chrono_running)
    {
        t = (millis() - _chrono_time_s);
    } else
    {
        t = _chrono_time_stop - _chrono_time_s;
    }
    xSemaphoreGive(_lock);
    return t;
}

auto clock_manager::get_chrono_running() const -> bool
{
    return _chrono_running;
}

auto clock_manager::get_chrono_start_ready() -> bool
{
    return !_chrono_running && _running_to.is_time_out();
}
