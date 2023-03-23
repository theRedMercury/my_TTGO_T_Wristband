#include "hardware/battery.hpp"
#include "hardware/hal.hpp"
#include "tools.hpp"

RTC_DATA_ATTR float voltage = 0.F;
RTC_DATA_ATTR int vref = 1100; // mV

void battery_manager::setup()
{
    DEBUG_PRINT("battery_manager setup : ");
    pinMode(CHARGE_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    esp_adc_cal_characteristics_t adc_chars;
    if (esp_adc_cal_characterize(ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, ADC_WIDTH_BIT_12, 1100, &adc_chars) ==
        ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        vref = adc_chars.vref;
    }
    DEBUG_PRINTLN("done");
}

void battery_manager::update()
{
    const float old_voltage = voltage;
    const bool old_is_charging = _is_charging;

    voltage =
        (static_cast<float>(analogRead(BATT_ADC_PIN)) / 4095.F) * 2.F * 3.3F * (static_cast<float>(vref) / 1000.0F);
    _is_charging = !static_cast<bool>(digitalRead(CHARGE_PIN));

    _state_change = tools::changed(voltage, old_voltage, 0.2F) || (old_is_charging != _is_charging);
}

void battery_manager::update_light()
{
    const float old_light = _light;
    _light = _is_charging ? _light + 0.01F : 0;

    if (old_light != _light)
    {
        analogWrite(LED_PIN, abs(sinf(_light) * 256.F));
    }
}

auto battery_manager::is_charging() const -> bool
{
    return _is_charging;
}

auto battery_manager::state_changed() const -> bool
{
    return _state_change;
}

auto battery_manager::get_voltage() const -> float
{
    return voltage;
}

auto battery_manager::get_percentage() const -> uint8_t
{
    return static_cast<uint8_t>(
        tools::clamp(((voltage - BATTERY_MIN_V) * 100.F / (BATTERY_MAX_V - BATTERY_MIN_V)), 0.F, 100.F));
}
