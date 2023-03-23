#pragma once

#include <Arduino.h>
#include <NTP.h>
#include <WiFiUdp.h>
#include <pcf8563.h>

#include "watch/watch_abs.hpp"

class ntp_manager final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup();
    auto sync_time() -> RTC_Date;

private:
    WiFiUDP _wifi_udp;
    NTP _ntp{_wifi_udp};
};
