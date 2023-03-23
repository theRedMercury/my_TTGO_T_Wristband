#include "wifi/ntp.hpp"
#include "tools.hpp"

void ntp_manager::setup()
{
    DEBUG_PRINT("ntp_manager setup : ");
    _ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120);
    _ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60);
    DEBUG_PRINTLN("done");
}

auto ntp_manager::sync_time() -> RTC_Date
{
    _ntp.begin();
    _ntp.update();
    RTC_Date datetime = RTC_Date(_ntp.year(), _ntp.month(), _ntp.day(), _ntp.hours(), _ntp.minutes(), _ntp.seconds());
    _ntp.stop();
    return datetime;
}
