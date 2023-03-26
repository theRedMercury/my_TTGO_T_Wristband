#include "watch/watch.hpp"
#include "hardware/eeprom.hpp"

#include <Wire.h>

RTC_DATA_ATTR uint8_t current_day = 0;

void watch_manager::setup()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK);

    input_m.setup();
    clk_m.setup();
    screen.setup();
    // keep screen of deepsleep to wake from timer
    screen.set_backlight(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER);
    ui_m.setup();

    xTaskCreatePinnedToCore(&update_gui, "u_gui", 4096 * 4, this, 6, &_handle_update_gui, 0);
    xTaskCreatePinnedToCore(&update_gui_sec, "u_gui_sec", 2048, this, 5, &_handle_update_gui_sec, 0);

    xTaskCreatePinnedToCore(&input_trigger, "input_trigger", 4096, this, 4, NULL, 1);
    xTaskCreatePinnedToCore(&update_mpu, "update_mpu", 4096, this, 3, &_handle_update_mpu, 1);
    xTaskCreatePinnedToCore(&update_bat_state, "u_bat", 4096, this, tskIDLE_PRIORITY, &_handle_update_bat_state, 1);
    xTaskCreatePinnedToCore(&update_wifi_pipeline, "u_wifi", 4096, this, 1, &_handle_update_wifi_pipeline, 1);
}

auto watch_manager::_time_to_light_sleep() -> bool
{
    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    const RTC_Date d = clk_m.get_clock_time();
    xSemaphoreGive(i2c_lock);

    return d.hour >= 7 && d.hour < 22;
}

auto watch_manager::go_to_sleep() -> bool
{
    if (bat_m.is_charging())
    {
        modem_sleep();
        return false;
    }

    if (ui_m.get_page() == ui_page::running || ui_m.get_page() == ui_page::chrono || clk_m.get_chrono_running() ||
        _time_to_light_sleep())
    {
        light_sleep();

        switch (esp_sleep_get_wakeup_cause())
        {
        case ESP_SLEEP_WAKEUP_EXT0:
            DEBUG_PRINTLN("Wakeup caused by external signal using RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            DEBUG_PRINTLN("Wakeup caused by external signal using RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            DEBUG_PRINTLN("Wakeup caused by timer");
            epprom_mem::store_steps_mem(mpu_m.get_steps());
            epprom_mem::store_step_time_mem(mpu_m.get_step_time());
            go_to_sleep();
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            DEBUG_PRINTLN("Wakeup caused by touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            DEBUG_PRINTLN("Wakeup caused by ULP program");
            break;
        default:
            DEBUG_PRINTLN("Wakeup was not caused by deep sleep:");
            break;
        }
        return true;
    }
    deep_sleep();
    return false; // useless
}

void watch_manager::normal_mode()
{
    DEBUG_PRINTLN("WAKE");
    _is_in_modem_sleep = false;
    set_need_update_ui(true);

    WiFi.setSleep(false);
    screen.wake_up();
    wifi_m.setup();

    update_step();

    ets_update_cpu_frequency(240);

    /*if (_handle_update_gui == NULL)
    {
        xTaskCreatePinnedToCore(&update_gui, "u_gui", 4096 * 4, this, 6, &_handle_update_gui, 0);
    }
    if (_handle_update_gui_sec == NULL)
    {
        xTaskCreatePinnedToCore(&update_gui_sec, "u_gui_sec", 2048, this, 5, &_handle_update_gui_sec, 0);
    }
    if (_handle_update_wifi_pipeline == NULL)
    {
        xTaskCreatePinnedToCore(&update_wifi_pipeline, "u_wifi", 4096, this, 1, &_handle_update_wifi_pipeline, 1);
    }*/
}

void watch_manager::modem_sleep(bool change_freq)
{
    // SAFE_DELETE_TASK(_handle_update_wifi_pipeline);
    // SAFE_DELETE_TASK(_handle_update_gui_sec);
    // SAFE_DELETE_TASK(_handle_update_gui);

    wifi_m.deactivate(false);
    WiFi.setSleep(true);
    screen.idle();

    if (change_freq)
    {
        ets_update_cpu_frequency(80);
    }
    _is_in_modem_sleep = true;
    DEBUG_PRINTLN("SLEEP");
}

void watch_manager::light_sleep()
{
    wifi_m.deactivate(false);
    WiFi.setSleep(true);
    screen.idle();

    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    const RTC_Date d = clk_m.get_clock_time();
    xSemaphoreGive(i2c_lock);
    const uint8_t c_min = d.minute;
    const uint8_t c_sec = d.second;

    int8_t min_sleep = 59 - c_min;
    if (min_sleep < 3)
    {
        min_sleep = 59 + min_sleep;
    }
    const ulong sec_sleep = (min_sleep * T_60) + (T_60 - c_sec);
    DEBUG_PRINTLN("ms : " + String(sec_sleep * SEC_IN_MS));
    esp_sleep_enable_timer_wakeup(sec_sleep * SEC_IN_MS * 1000);

    /* Enable sleep fct */
    pinMode(IMU_INT2_PIN, GPIO_MODE_INPUT);
    DEBUG_PRINTLN("LIGHT SLEEP");
    // esp_sleep_enable_timer_wakeup(1000000 * 60 * 30); // 30 min
    esp_sleep_enable_ext0_wakeup(TP_PIN, 1); //  | GPIO_SEL_39
    DEBUG_PRINTLN("1");
    esp_light_sleep_start();
    DEBUG_PRINTLN("LIGHT SLEEP WAKEUP");
}

void watch_manager::deep_sleep()
{
    SAFE_DELETE_TASK(_handle_update_wifi_pipeline);
    SAFE_DELETE_TASK(_handle_update_mpu);
    SAFE_DELETE_TASK(_handle_update_bat_state);
    SAFE_DELETE_TASK(_handle_update_gui_sec);
    SAFE_DELETE_TASK(_handle_update_gui);

    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    const RTC_Date d = clk_m.get_clock_time();
    xSemaphoreGive(i2c_lock);

    screen.deep_sleep();
    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    mpu_m.light_sleep();
    xSemaphoreGive(i2c_lock);

    wifi_m.deactivate();

    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    clk_m.deep_sleep();
    xSemaphoreGive(i2c_lock);

    /* Enable sleep fct */
    pinMode(IMU_INT2_PIN, GPIO_MODE_INPUT);

    delay(250);

    const uint8_t c_hour = d.hour;
    const uint8_t c_min = d.minute;
    const uint8_t c_sec = d.second;
    int8_t min_sleep = 59 - c_min;
    if (min_sleep < 3)
    {
        min_sleep = 59 + min_sleep;
    }
    const ulong sec_sleep = (min_sleep * T_60) + (T_60 - c_sec);

    // Auto Wake up at 7h AM (sleep sec + (hour left * 60min * 60 sec * SEC_IN_MS))
    esp_sleep_enable_timer_wakeup(((sec_sleep * SEC_IN_MS) + (((23 - c_hour) + 6) * T_60 * T_60 * SEC_IN_MS)) * 1000);

    esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH); //  | GPIO_SEL_39
    esp_deep_sleep_start();
    /* nothing more here*/
}

auto watch_manager::get_need_update_ui() const -> bool
{
    return _need_update_ui;
}

void watch_manager::set_need_update_ui(bool update)
{
    _need_update_ui = update;
}

void watch_manager::update_gui(void* param)
{
    watch_manager* watch = reinterpret_cast<watch_manager*>(param);

    watch->ui_m.handle_ui();

    while (true)
    {
        const input_type it = watch->input_m.get_pressed_type();
        switch (it)
        {
        case input_type::LONG_PRESS:
            switch (watch->ui_m.get_page())
            {
            case ui_page::main:
                if (watch->wifi_m.is_on())
                {
                    watch->wifi_m.deactivate(true);
                } else
                {
                    watch->wifi_m.activate();
                }
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                break;

            case ui_page::running:
                xSemaphoreTake(watch->i2c_lock, portMAX_DELAY);
                watch->mpu_m.reset_step();
                xSemaphoreGive(watch->i2c_lock);
                watch->set_need_update_ui(true);
                break;

            case ui_page::chrono:
                if (watch->clk_m.get_chrono_running())
                {
                    watch->clk_m.stop_chrono();
                } else
                {
                    watch->clk_m.reset_chrono();
                }
                break;

            default:
                break;
            }
            break;

        case input_type::PRESS:
            watch->ui_m.next_page();
            break;

        default:
            break;
        }

        if (watch->get_need_update_ui() || watch->ui_m.get_need_redraw())
        {
            watch->set_need_update_ui(false);
            watch->ui_m.handle_ui();
        }
        vTaskDelay(33 / portTICK_PERIOD_MS);
    }
}

void watch_manager::update_gui_sec(void* param)
{
    watch_manager* watch = reinterpret_cast<watch_manager*>(param);
    bool t = true;
    uint8_t update_sec = 0; // 0 - 4
    vTaskDelay((1 * SEC_IN_MS) / portTICK_PERIOD_MS);
    while (true)
    {
        if ((update_sec % 2) == 0)
        {
            watch->ui_m.update_sec(t);
            t = !t;
        }

        watch->ui_m.update_wifi(update_sec);
        update_sec = update_sec <= 3 ? update_sec + 1 : 1;

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void watch_manager::input_trigger(void* param)
{
    watch_manager* watch = reinterpret_cast<watch_manager*>(param);
    while (true)
    {
        bool read_input = true;
        vTaskDelay(100 / portTICK_PERIOD_MS);
        while (read_input)
        {
            watch->input_m.update();
            vTaskDelay(88 / portTICK_PERIOD_MS);
            read_input = !watch->input_m.is_time_out();
        }

        if (!watch->go_to_sleep())
        {
            /* here modem sleep */
            vTaskDelay(250 / portTICK_PERIOD_MS);
            watch->input_m.reset_to();

            read_input = true;
            while (read_input)
            {
                watch->input_m.update();
                read_input = watch->input_m.get_pressed_type() != input_type::PRESS;
                vTaskDelay(150 / portTICK_PERIOD_MS);
            }
        } else
        {
            /* light_sleep wakeup */
            vTaskDelay(250 / portTICK_PERIOD_MS);
            watch->input_m.reset_to();
        }

        watch->normal_mode();
    }
}

void watch_manager::update_bat_state(void* param)
{
    watch_manager* watch = reinterpret_cast<watch_manager*>(param);

    vTaskDelay(600 / portTICK_PERIOD_MS);

    watch->bat_m.setup();
    uint8_t counter = 0;

    vTaskDelay(400 / portTICK_PERIOD_MS);

    while (true)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS); // 1 sec
        if (counter == 0)
        {
            watch->bat_m.update();

            if (watch->bat_m.state_changed())
            {
                watch->set_need_update_ui(true);
                if (watch->bat_m.is_charging())
                {
                    watch->ui_m.set_charging();
                }
            }
        }
        counter = counter > 100 ? 0 : counter + 1;
        watch->bat_m.update_light();
    }
}

void watch_manager::update_mpu(void* param)
{
    watch_manager* watch = reinterpret_cast<watch_manager*>(param);
    vTaskDelay((1 * SEC_IN_MS) / portTICK_PERIOD_MS);
    xSemaphoreTake(watch->i2c_lock, portMAX_DELAY);
    watch->mpu_m.setup();
    xSemaphoreGive(watch->i2c_lock);
    while (true)
    {
        vTaskDelay((watch->is_in_modem_sleep() ? 500 : 33) / portTICK_PERIOD_MS);
        xSemaphoreTake(watch->i2c_lock, portMAX_DELAY);
        watch->mpu_m.update();
        xSemaphoreGive(watch->i2c_lock);
    }
}

void watch_manager::update_step()
{
    const uint8_t c_day = current_day;

    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    current_day = clk_m.get_clock_time().day;
    xSemaphoreGive(i2c_lock);

    /* New Day */
    if (c_day != current_day && c_day != 0 && current_day != 0)
    {
        DEBUG_PRINTLN("New day : " + String(c_day) + " !=" + String(current_day));
        xSemaphoreTake(i2c_lock, portMAX_DELAY);
        mpu_m.reset_step();
        xSemaphoreGive(i2c_lock);
        set_need_update_ui(true);
    }
}

void watch_manager::update_wifi_pipeline(void* param)
{
    watch_manager* watch = reinterpret_cast<watch_manager*>(param);
    delay_time_out to_pipeline{1};
    watch->wifi_m.setup();
    watch->ntp_m.setup();

    while (true)
    {
        watch->wifi_m.update();
        if (watch->wifi_m.is_state_change())
        {
            if (watch->wifi_m.is_connected() && to_pipeline.is_time_out())
            {
                xSemaphoreTake(watch->i2c_lock, portMAX_DELAY);
                watch->clk_m.set_time(watch->ntp_m.sync_time());
                xSemaphoreGive(watch->i2c_lock);

                watch->update_step();

                if (weather_api::update())
                {
                    to_pipeline.reset_delay(SEC_IN_MS * T_60 * 30); // 30min
                } else
                {
                    to_pipeline.reset_delay(SEC_IN_MS * 10); // 10 sec
                }
            }
            watch->set_need_update_ui(true);
        }
        vTaskDelay((2 * SEC_IN_MS) / portTICK_PERIOD_MS);
    }
}
