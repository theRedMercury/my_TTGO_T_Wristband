
#include "wifi/weather.hpp"
#include "tools.hpp"
#include "wifi/_personal_conf.hpp"

#include <Arduino_JSON.h>
#include <WiFi.h>

RTC_DATA_ATTR char weather_api::current_str[18] = "...";
RTC_DATA_ATTR float weather_api::temp = 0;
RTC_DATA_ATTR float weather_api::temp_feel = 0;

RTC_DATA_ATTR float weather_api::temp_min = 0;
RTC_DATA_ATTR float weather_api::temp_max = 0;
RTC_DATA_ATTR uint16_t weather_api::pressure = 0;
RTC_DATA_ATTR uint8_t weather_api::humidity = 0;
RTC_DATA_ATTR float weather_api::wind_speed = 0;
RTC_DATA_ATTR int16_t weather_api::wind_degs = 0;
RTC_DATA_ATTR uint8_t weather_api::clouds = 0;

static char const* degs_str[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};

auto weather_api::update() -> bool
{
    if (WiFi.status() == WL_CONNECTED)
    {
        JSONVar result = JSON.parse(tools::http_get_request(openweather_url));

        // JSON.typeof(jsonVar) can be used to get the type of the var
        if ((JSON.typeof(result) == "undefined") || result.length() == 0)
        {
            DEBUG_PRINTLN("Parsing input failed!");
            return false;
        }
        String current = JSON.stringify(result["weather"][0]["description"]);
        if (current.length() > 2)
        {
            current = current.substring(1, current.length() - 1);
        }
        current.toCharArray(current_str, 18, 0);
        temp = result["main"]["temp"];
        temp_min = result["main"]["temp_min"];
        temp_max = result["main"]["temp_max"];
        temp_feel = result["main"]["feels_like"];
        pressure = result["main"]["pressure"];
        humidity = result["main"]["humidity"];
        wind_speed = result["wind"]["speed"];
        wind_degs = result["wind"]["deg"];
        clouds = tools::clamp(static_cast<uint8_t>(result["clouds"]["all"]), static_cast<uint8_t>(0),
                              static_cast<uint8_t>(100));
        return true;
    }
    return false;
}

auto weather_api::get_degs_str(int16_t cap) -> char const*
{
    switch (cap)
    {
    case 0 ... 22:
        return degs_str[0];
    case 23 ... 68:
        return degs_str[1];
    case 69 ... 114:
        return degs_str[2];
    case 115 ... 160:
        return degs_str[3];
    case 161 ... 206:
        return degs_str[4];
    case 207 ... 252:
        return degs_str[5];
    case 253 ... 298:
        return degs_str[6];
    case 299 ... 344:
        return degs_str[7];
    default:
        return degs_str[0];
    }
}
