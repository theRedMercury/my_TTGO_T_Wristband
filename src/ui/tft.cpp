#include "ui/tft.hpp"
#include "watch/watch.hpp"
#include "wifi/weather.hpp"

constexpr auto MAIN_CLOCK_COLOR = 0x0000;
constexpr uint8_t CLOCK_Y_POS = 28;

TFT_eSPI tft_screen::_tft = TFT_eSPI();

void tft_screen::setup()
{
    DEBUG_PRINT("tft_screen setup : ");
    // const uint8_t channel = 0;
    // ledcSetup(channel, 5000, 8);
    // ledcAttachPin(TFT_BL, channel);
    // ledcWrite(channel, 255);

    //_tft.writecommand(ST7735_SWRESET); /* soft reset */
    // delay(150);
    _tft.writecommand(ST7735_DISPON); /* display on */
    delay(120);
    _tft.writecommand(ST7735_SLPOUT); /* sleep out */
    delay(120);
    // _tft.writecommand(ST7735_IDMOFF);

    _tft.init();
    _tft.setRotation(0);
    _tft.setSwapBytes(true);
    _tft.fillScreen(TFT_BLACK);
    // _tft.setTextDatum(TL_DATUM);
    // _tft.writecommand(ST7735_COLMOD);
    // _tft.writedata(0x05); // 3 / 5 / 6 (color depth) RGB666
    _is_in_idle = false;
    DEBUG_PRINTLN("done");
}

void tft_screen::set_backlight(bool on) const
{
    digitalWrite(TFT_BL, on ? HIGH : LOW);
}

void tft_screen::clear()
{
    _tft.fillScreen(TFT_BLACK);
    _y_time_pos = 0;
}

void tft_screen::idle()
{
    // ledcSetup(8, 5000, 8);    // 0-15, 5000, 8
    // ledcAttachPin(TFT_BL, 8); // TFT_BL, 0 - 15
    // ledcWrite(8, 1);          // 0-15, 0-255 (with 8 bit resolution)
    set_backlight(false);

    _tft.writecommand(ST7735_IDMON); /* IDLE ON */
    delay(120);
    _is_in_idle = true;
}
void tft_screen::deep_sleep()
{
    for (int i = 0; i < TFT_HEIGHT / 2; ++i)
    {
        _tft.drawFastHLine(0, i, TFT_WIDTH, TFT_BLACK);
        _tft.drawFastHLine(0, TFT_HEIGHT - i, TFT_WIDTH, TFT_BLACK);

        _tft.drawFastHLine(0, i + 1, TFT_WIDTH, TFT_WHITE);
        _tft.drawFastHLine(0, TFT_HEIGHT - i - 1, TFT_WIDTH, TFT_WHITE);
        delay(2);
    }
    for (int i = 0; i < TFT_WIDTH / 2; ++i)
    {
        _tft.drawFastVLine(i, 76, 8, TFT_BLACK);
        _tft.drawFastVLine(TFT_WIDTH - i, 76, 8, TFT_BLACK);
        delay(2);
    }

    set_backlight(false);
    //_tft.writecommand(ST7735_SWRESET); /* soft reset */
    // delay(150);
    _tft.writecommand(ST7735_SLPIN); /* sleep in */
    delay(120);
    _tft.writecommand(ST7735_DISPOFF); /* display off */
    delay(120);
    _is_in_idle = false;
}

void tft_screen::wake_up()
{
    if (_is_in_idle)
    {
        // ledcSetup(8, 5000, 8);    // 0-15, 5000, 8
        // ledcAttachPin(TFT_BL, 8); // TFT_BL, 0 - 15
        // ledcWrite(8, 255);        // 0-15, 0-255 (with 8 bit resolution)
        set_backlight(true);
        _tft.writecommand(ST7735_IDMOFF); /* IDLE OFF */
        delay(120);
    } else
    {
        setup();
    }
}

void tft_screen::play_anim_transition()
{
    for (uint8_t i = TFT_WIDTH; i > 1; i--)
    {
        _tft.drawFastVLine(i, 0, TFT_HEIGHT, TFT_BLACK);
        _tft.drawFastVLine(i - 1, 0, TFT_HEIGHT, TFT_WHITE);
        delay(1);
    }
    _tft.fillScreen(TFT_BLACK);
    _watch->clk_m.reset_to();
}

void tft_screen::show_main_page(const uint8_t hour, const uint8_t minute, const uint8_t day, const uint8_t month,
                                bool utc)
{
    _show_header(day, month);
    _draw_clock(hour, minute);

    if (_watch->clk_m.get_chrono_running())
    {
        _tft.drawTriangle(0, 100, 10, 100, 5, 108, TFT_ORANGE);
        _tft.drawTriangle(0, 116, 10, 116, 5, 108, TFT_ORANGE);
    }

    _draw_basic_wheater();

    char temp[5]{' ', ' ', ' ', ' ', ' '};
    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    sprintf(temp, "%d", _watch->mpu_m.get_steps());
    _tft.drawRightString(temp, 80, 140, 2);
}

void tft_screen::_show_header(const uint8_t day, const uint8_t month)
{
    if (_d != day || _mo != month)
    {
        _d = day;
        _mo = month;
        _tft.fillRect(0, 0, TFT_WIDTH, 20, TFT_BLACK);
    }
    /* Wifi */
    _draw_wifi_arc(0);

    /* Date */
    _draw_date(day, month);

    /* Battery */
    _draw_battery_arc();
}

inline void tft_screen::_draw_battery_arc()
{
    const uint8_t bat_percent = _watch->bat_m.get_percentage();
    int32_t p = bat_percent * 3.6F;
    p = p >= 360 ? 359 : p;
    p = p == 0 ? 1 : p;
    const int32_t r_end_bat = p > 180 ? (p % 180) : 180 + (p % 180);

    uint32_t color_bat = TFT_WHITE;
    if (bat_percent <= 9)
    {
        color_bat = TFT_RED;
    }
    if (_watch->bat_m.is_charging())
    {
        color_bat = p == 359 ? TFT_GREEN : TFT_ORANGE;
    }

    _tft.fillCircle(70, 10, 8, TFT_BLACK);
    _tft.drawSmoothArc(70, 10, 7, 7, 180, r_end_bat, color_bat, TFT_BLACK, true);
    if (bat_percent <= 9)
    {
        char t_pb[1]{' '};
        sprintf(t_pb, "%d", bat_percent);
        _tft.setTextColor(TFT_RED, TFT_BLACK);
        _tft.drawString(t_pb, 148, 8, 1);
    }
}

inline void tft_screen::_draw_wifi_arc(const uint8_t counter)
{
    const wifi_signal w_s = _watch->wifi_m.get_strenght();
    const uint8_t yp = 14;
    const int32_t x = 2;
    if (_watch->wifi_m.is_on())
    {

        int32_t r = 5;
        const int32_t r_start = 185;
        const int32_t r_end = 265;

        _tft.drawSmoothCircle(x, yp, 1, ((counter == 4 && w_s == wifi_signal::S_ON) ? TFT_ORANGE : SEG7_BACKGROUND),
                              TFT_BLACK);
        if (w_s >= wifi_signal::S_LOW)
        {
            _tft.drawSmoothCircle(x, yp, 1, TFT_WHITE, TFT_BLACK);
        }

        for (uint8_t i = 1; i <= 3; ++i)
        {
            _tft.drawSmoothArc(x, yp, r, r - 1, r_start, r_end,
                               ((counter == i && w_s == wifi_signal::S_ON) ? TFT_ORANGE : SEG7_BACKGROUND), TFT_BLACK,
                               true);
            r += 4;
        }
        r = 5;

        if (w_s >= wifi_signal::S_FAIR)
        {
            _tft.drawSmoothArc(x, yp, r, r - 1, r_start, r_end, TFT_WHITE, TFT_BLACK, true);
        }
        r += 4;
        if (w_s >= wifi_signal::S_GOOD)
        {
            _tft.drawSmoothArc(x, yp, r, r - 1, r_start, r_end, TFT_WHITE, TFT_BLACK, true);
        }
        r += 4;
        if (w_s >= wifi_signal::S_EXCEL)
        {
            _tft.drawSmoothArc(x, yp, r, r - 1, r_start, r_end, TFT_WHITE, TFT_BLACK, true);
        }
    } else
    {
        _tft.fillRect(0, 0, 21, 21, TFT_BLACK);
    }
}

void tft_screen::_draw_date(const uint8_t day, const uint8_t month)
{
    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    char date_s[2]{' ', ' '};
    sprintf(date_s, "%d", day);
    _tft.drawRightString(date_s, 36, 2, 2);

    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    _tft.drawCentreString("/", 40, 2, 2);

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    char date_m[2]{' ', ' '};
    sprintf(date_m, "%02d", tools::clamp(month, (uint8_t)1, (uint8_t)12));
    _tft.drawString(date_m, 44, 2, 2);
}

void tft_screen::_draw_clock(const uint8_t hour, const uint8_t minute)
{
    uint8_t xpos = 0;
    if (_h != hour || _m != minute)
    {
        _h = hour;
        _m = minute;
        _tft.fillRect(0, 28, TFT_WIDTH, 130, TFT_BLACK);
    }
    load_font(MAIN_CLOCK_FONT, true);

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (hour < 10)
    {
        xpos += _tft.drawNumber(0, xpos, CLOCK_Y_POS);
    }
    xpos += _tft.drawNumber(hour, xpos, CLOCK_Y_POS);
    _sec_dot_xpos = xpos;
    xpos += display_sec_dot(true, true);
    xpos = 22;

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (minute < 10)
    {
        xpos += _tft.drawNumber(0, xpos, CLOCK_Y_POS + 48);
    }
    _tft.drawNumber(minute, xpos, CLOCK_Y_POS + 48);

    load_font(nullptr, false);
}

void tft_screen::show_battery_page(float voltage, uint8_t percentage, bool charging)
{
    const int16_t p = static_cast<int16_t>((static_cast<float>(percentage) / 100.F) * static_cast<float>(TFT_HEIGHT));
    if (_percent_bat != p)
    {
        _percent_bat = p;
        _tft.fillScreen(TFT_BLACK);

        _tft.fillSmoothRoundRect(0, TFT_HEIGHT - p, TFT_WIDTH, p, 0, 0x0461, 0x02C0);
        _tft.setTextColor(TFT_WHITE);
        _tft.drawCentreString(String(percentage) + "%", 40, 60, 4);

        char voltageString[5]{' ', ' ', ' ', ' ', ' '};
        sprintf(voltageString, "%1.2fv", voltage);
        _tft.drawCentreString(voltageString, 40, 142, 1);
    }
}

void tft_screen::show_weather_page()
{
    /* Weather */
    char temp[5]{' ', ' ', ' ', ' ', ' '};

    /* Temp OUT */
    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    sprintf(temp, "%2.1fc", weather_api::temp);
    _tft.drawRightString(temp, 63, 0, 4);

    /* Temp Feel */
    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    sprintf(temp, "%2.1fc", weather_api::temp_feel);
    _tft.drawRightString(temp, 63, 20, 4);

    _tft.drawFastHLine(0, 43, 80, TFT_WHITE);

    /* Temp min / max */
    char temp_n[4]{' ', ' ', ' ', ' '};
    sprintf(temp_n, "%2.1f", weather_api::temp_min);
    _tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
    _tft.drawString(temp_n, 0, 44, 2);
    sprintf(temp_n, "%2.1f", weather_api::temp_max);
    _tft.setTextColor(TFT_RED, TFT_BLACK);
    _tft.drawRightString(temp_n, 80, 44, 2);

    /* Cloud */
    _tft.fillSmoothCircle(30, 66, 6, TFT_WHITE, TFT_BLACK);
    _tft.fillSmoothCircle(48, 66, 6, TFT_WHITE, TFT_BLACK);
    _tft.fillCircle(40, 62, 10, TFT_WHITE);
    _tft.setTextColor(TFT_BLACK, TFT_BLACK);
    if (weather_api::clouds < 100)
    {
        _tft.drawCentreString(String(weather_api::clouds), 41, 55, 2);
    }

    /* Wind */
    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    _tft.drawCentreString(weather_api::get_degs_str(
                              tools::clamp(weather_api::wind_degs, static_cast<int16_t>(0), static_cast<int16_t>(259))),
                          30, 76, 2);
    sprintf(temp_n, "%2.1f", weather_api::wind_speed);
    _tft.drawCentreString(temp_n, 50, 76, 2);

    /* INFO */
    const String c_str = String(weather_api::current_str);
    _tft.drawCentreString(tools::split_str(c_str, 0, ' '), 40, 98, 2);
    _tft.drawCentreString(tools::split_str(c_str, 1, ' '), 40, 112, 2);

    /* humidity / pressure */
    sprintf(temp_n, "%d", weather_api::humidity);
    _tft.drawString(temp_n, 0, 132, 2);
    _tft.drawString("%", 0, 148, 1);
    sprintf(temp_n, "%d", weather_api::pressure);
    _tft.drawRightString(temp_n, 80, 132, 2);
    _tft.drawRightString("hPa", 80, 148, 1);
}

void tft_screen::show_compass_page()
{
    const int16_t bearing = _watch->mpu_m.get_heading();
    char bearingText[5]{' ', ' ', ' ', ' ', ' '};
    if (bearing >= 0)
    {
        sprintf(bearingText, "%03d", bearing);
    } else
    {
        sprintf(bearingText, "---");
    }

    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);

    // DEBUG_PRINTLN(_watch->mpu_m.get_imu()->gx);a
    // DEBUG_PRINTLN(_watch->mpu_m.get_imu()->gy);
    // DEBUG_PRINTLN(_watch->mpu_m.get_imu()->gz);
    //_watch->mpu_m.get_heading();
    //-tools::map(_watch->mpu_m.get_imu()->accel_y_mps2(), (float)-20, (float)20, -180, 180);

    const float angle_a = static_cast<float>(bearing) * DEG_TO_RAD;
    const float angle_b = 0; //_watch->mpu_m.get_pitch()* DEG_TO_RAD;
    //-tools::map(_watch->mpu_m.get_imu()->accel_z_mps2(), (float)-20, (float)20, -180, 180);
    const float angle_c = _watch->mpu_m.get_roll() * DEG_TO_RAD;
    //_watch->mpu_m.get_imu()->pry_y() * RAD_TO_DEG;
    // tools::map(_watch->mpu_m.get_imu()->accel_x_mps2(), (float)-20, (float)20, -180, 180);

    /*
    x' = x*cos a - y*sin a;
    y' = x*sin a + y*cos a;

    x'' = x'*cos b - z' * sin b;
    z'' = x'*sin b + z' * cos b;

    y''' = y'' * cos c - z'' * sin c
    z''' = y'' * sin c + z'' * cos c
    */
    int16_t points[5][2];
    float rotated_3d_points[5][3];
    for (uint8_t i = 0; i < 5; ++i)
    {
        // X
        // Z
        // Y
        const float x = _orig_points_c[i][0];
        const float y = _orig_points_c[i][1];
        const float z = _orig_points_c[i][2];

        rotated_3d_points[i][0] = x * cos(angle_a) - y * sin(angle_a);
        rotated_3d_points[i][1] = x * sin(angle_a) + y * cos(angle_a);
        rotated_3d_points[i][2] = _orig_points_c[i][2];

        const float xx = rotated_3d_points[i][0];
        const float yy = rotated_3d_points[i][1];
        const float zz = rotated_3d_points[i][2];
        rotated_3d_points[i][0] = xx * cos(angle_b) - zz * sin(angle_b);
        rotated_3d_points[i][1] = yy;
        rotated_3d_points[i][2] = xx * sin(angle_b) + zz * cos(angle_b);

        const float xxx = rotated_3d_points[i][0];
        const float yyy = rotated_3d_points[i][1];
        const float zzz = rotated_3d_points[i][2];
        rotated_3d_points[i][0] = xxx;
        rotated_3d_points[i][1] = yyy * cos(angle_c) - zzz * sin(angle_c);
        rotated_3d_points[i][2] = yyy * sin(angle_c) + zzz * cos(angle_c);

        rotated_3d_points[i][2] = rotated_3d_points[i][2] + z_offset;

        // project 3d points into 2d space with perspective divide -- 2D x = x/z,   2D y = y/z
        points[i][0] = round(40 + rotated_3d_points[i][0] / rotated_3d_points[i][2] * cube_size);
        points[i][1] = round(80 + rotated_3d_points[i][1] / rotated_3d_points[i][2] * cube_size);
    }

    clear();
    _tft.drawLine(points[0][0], points[0][1], points[1][0], points[1][1], TFT_RED);
    _tft.drawLine(points[0][0], points[0][1], points[2][0], points[2][1], TFT_GREEN);
    _tft.drawLine(points[0][0], points[0][1], points[3][0], points[3][1], TFT_BLUE);
    _tft.drawLine(points[0][0], points[0][1], points[4][0], points[4][1], TFT_ORANGE);

    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    _tft.drawCentreString(bearingText, 40, 2, 4);

    // sprintf(bearingText, "%03d", _watch->mpu_m.get_temp());
    // _tft.drawCentreString(bearingText, 40, 130, 2);

    int32_t xp = 2;

    //_tft.fillRect(0, 120, TFT_WIDTH, 40, TFT_BLACK);

    for (int16_t i = -40; i < 400; ++i)
    {
        if ((bearing - 10) <= i && (bearing + 10) >= i)
        {
            if ((i % 10) == 0)
            {
                if (i == 0 || i == 90 || i == 180 || i == 270 || i == 360)
                {
                    _tft.drawCentreString(weather_api::get_degs_str(i), xp - 4, 120, 4);

                } else
                {
                    _tft.drawFastVLine(xp, 120, 20, TFT_ORANGE);
                }

            } else
            {
                if (i == 45 || i == 135 || i == 225 || i == 305)
                {
                    _tft.drawCentreString(weather_api::get_degs_str(i), xp - 2, 120, 2);

                } else
                {
                    _tft.drawFastVLine(xp, 125, 10, TFT_WHITE);
                }
            }

            xp += 4;
        }
    }
    //_tft.drawCentreString(bearingText, 80, 42, 4);
}

void tft_screen::show_running_page()
{
    // 10000 steps
    const ulong step = tools::clamp(_watch->mpu_m.get_steps(), (ulong)0, (ulong)99999999);
    const int16_t step_arc = ((static_cast<float>(step) / 10000.F) * 180.F) + 90;
    _tft.drawSmoothArc(40, 40, 38, 34, 90, 270, 0xA37200, TFT_BLACK, true);
    _tft.drawSmoothArc(40, 40, 38, 34, 90, (step_arc > 91 ? step_arc : 91), TFT_ORANGE, TFT_BLACK, true);

    char temp[10]{' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    sprintf(temp, "%03d", step);
    _tft.drawCentreString(temp, 40, 48, 4);

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    sprintf(temp, "%d min",
            tools::clamp((ulong)(_watch->mpu_m.get_step_time() / (ulong)60000), (ulong)0, (ulong)99999999));
    _tft.drawCentreString(temp, 40, 100, 2);
}

void tft_screen::show_chrono_page()
{
    if (_watch->clk_m.get_chrono_start_ready())
    {
        _watch->clk_m.start_chrono();
    }

    const ulong tm = _watch->clk_m.get_chrono_time();
    const uint8_t h = tools::clamp(static_cast<int>((tm / (SEC_IN_MS * T_60 * T_60)) % 24), 0, 23);
    const uint8_t m = tools::clamp(static_cast<int>((tm / (SEC_IN_MS * T_60)) % T_60), 0, 59);
    const uint8_t s = tools::clamp(static_cast<int>((tm / SEC_IN_MS) % T_60), 0, 59);
    char temp[5]{' ', ' ', ' ', ' ', ' '};

    if (h > 0)
    {
        _tft.setTextColor(TFT_WHITE, TFT_BLACK);
        sprintf(temp, "%02d", h);
        _tft.drawCentreString(temp, 40, 22, 4);
    }

    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    sprintf(temp, "%02d.%02d", m, s);
    _tft.drawCentreString(temp, 40, 48, 4);

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    sprintf(temp, "%03d", tools::clamp(static_cast<int>(tm % SEC_IN_MS), 0, 999));
    _tft.drawCentreString(temp, 40, 76, 2);
}

void tft_screen::show_mpu_page()
{
    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);

    // DEBUG_PRINTLN(_watch->mpu_m.get_imu()->gx);a
    // DEBUG_PRINTLN(_watch->mpu_m.get_imu()->gy);
    // DEBUG_PRINTLN(_watch->mpu_m.get_imu()->gz);
    //_watch->mpu_m.get_heading();
    const float angle_a = tools::map(_watch->mpu_m.get_accelX(), (float)-2, (float)2, -180, 180) * DEG_TO_RAD;
    const float angle_b = tools::map(_watch->mpu_m.get_accelY(), (float)-2, (float)2, -180, 180) * DEG_TO_RAD;
    const float angle_c = tools::map(_watch->mpu_m.get_accelZ(), (float)-2, (float)2, -180, 180) * DEG_TO_RAD;

    /*
    x' = x*cos a - y*sin a;
    y' = x*sin a + y*cos a;

    x'' = x'*cos b - z' * sin b;
    z'' = x'*sin b + z' * cos b;

    y''' = y'' * cos c - z'' * sin c
    z''' = y'' * sin c + z'' * cos c
    */
    int16_t points[4][2];
    float rotated_3d_points[4][3];
    for (uint8_t i = 0; i < 4; ++i)
    {
        // X
        // Z
        // Y
        const float x = _orig_points[i][0];
        const float y = _orig_points[i][1];
        const float z = _orig_points[i][2];
        rotated_3d_points[i][0] = x * cos(angle_a) - y * sin(angle_a);
        rotated_3d_points[i][1] = x * sin(angle_a) + y * cos(angle_a);
        rotated_3d_points[i][2] = _orig_points[i][2];

        const float xx = rotated_3d_points[i][0];
        const float yy = rotated_3d_points[i][1];
        const float zz = rotated_3d_points[i][2];
        rotated_3d_points[i][0] = xx * cos(angle_b) - zz * sin(angle_b);
        rotated_3d_points[i][1] = yy;
        rotated_3d_points[i][2] = xx * sin(angle_b) + zz * cos(angle_b);

        const float xxx = rotated_3d_points[i][0];
        const float yyy = rotated_3d_points[i][1];
        const float zzz = rotated_3d_points[i][2];
        rotated_3d_points[i][0] = xxx;
        rotated_3d_points[i][1] = yyy * cos(angle_c) - zzz * sin(angle_c);
        rotated_3d_points[i][2] = yyy * sin(angle_c) + zzz * cos(angle_c);

        rotated_3d_points[i][2] = rotated_3d_points[i][2] + z_offset;

        // project 3d points into 2d space with perspective divide -- 2D x = x/z,   2D y = y/z
        points[i][0] = round(40 + rotated_3d_points[i][0] / rotated_3d_points[i][2] * cube_size);
        points[i][1] = round(80 + rotated_3d_points[i][1] / rotated_3d_points[i][2] * cube_size);
    }

    clear();
    _tft.drawLine(points[0][0], points[0][1], points[1][0], points[1][1], TFT_RED);
    _tft.drawLine(points[0][0], points[0][1], points[2][0], points[2][1], TFT_GREEN);
    _tft.drawLine(points[0][0], points[0][1], points[3][0], points[3][1], TFT_BLUE);
    return;

    int16_t x = 0; // tools::map(_watch->mpu_m.get_imu()->accel_x_mps2(), (float)-10, (float)10, 0, 79);
    int16_t y = 0; // tools::map(_watch->mpu_m.get_imu()->accel_y_mps2(), (float)-10, (float)10, 0, 79);
    int16_t z = 0; // tools::map(_watch->mpu_m.get_imu()->accel_z_mps2(), (float)-10, (float)10, 0, 79);

    _tft.drawFastVLine(_y_time_pos, x, 1, TFT_RED);
    _tft.drawFastVLine(_y_time_pos, y, 1, TFT_GREEN);
    _tft.drawFastVLine(_y_time_pos, z, 1, TFT_BLUE);

    _y_time_pos++;
    if (_y_time_pos >= 160)
    {
        _y_time_pos = 0;
    }

    uint8_t yy = _y_time_pos;
    for (uint8_t i = 0; i < 10; ++i)
    {

        if (yy >= 160)
        {
            yy = 0;
        }
        _tft.drawFastVLine(yy, 0, 79, TFT_BLACK);
        yy++;
    }
}

void tft_screen::_draw_basic_wheater()
{
    _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    char temp[5]{' ', ' ', ' ', ' ', ' '};
    sprintf(temp, "%2.1fc", weather_api::temp);
    _tft.drawString(temp, 0, 126, 2);

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    sprintf(temp, "%2.1fc", weather_api::temp_feel);
    _tft.drawString(temp, 0, 140, 2);
}

void tft_screen::draw_wifi_signal(uint8_t counter)
{
    _draw_wifi_arc(counter);
}

uint16_t tft_screen::display_sec_dot(bool color, bool utc)
{
    if (color)
    {
        _tft.setTextColor(0x0821, TFT_BLACK);
    } else
    {
        if (utc)
        {
            _tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        } else
        {
            _tft.setTextColor(0xFBE0, TFT_BLACK);
        }
    }
    return static_cast<uint16_t>(_tft.drawString(".", _sec_dot_xpos, CLOCK_Y_POS));
}

void tft_screen::load_font(const uint8_t* array, bool load)
{
    if (load)
    {
        _tft.loadFont(MAIN_CLOCK_FONT);
    } else
    {
        _tft.unloadFont();
    }
}
