#pragma once

#include <Arduino.h>

/* Input */
constexpr auto TP_PIN = GPIO_NUM_33;
constexpr auto TP_PIN_POWER = GPIO_NUM_25;

/* I2C */
constexpr auto I2C_SDA_PIN = GPIO_NUM_21;
constexpr auto I2C_SCL_PIN = GPIO_NUM_22;
constexpr auto I2C_CLOCK = 400000;

/* Interrupt */
constexpr auto IMU_INT2_PIN = GPIO_NUM_39;
constexpr auto IMU_INTM_PIN = GPIO_NUM_37;
constexpr auto IMU_READY_PIN = GPIO_NUM_36;

/* IMU (MPU) */
constexpr auto MPU_INT_PIN = GPIO_NUM_38;
constexpr auto MPU_ADDR = 0x69;
constexpr auto GYRO_CALIBRATION_BASE_ADDRESS = 0x10;
constexpr auto MAG_CALIBRATION_BASE_ADDRESS = 0x20;
constexpr auto ACCEL_CALIBRATION_BASE_ADDRESS = 0x30;

/* RTC */
constexpr auto RTC_INT_PIN = GPIO_NUM_34;

/* Battery */
constexpr float BATTERY_MIN_V = 3.0F;
constexpr float BATTERY_MAX_V = 4.1F;
constexpr auto LED_PIN = GPIO_NUM_4;
constexpr auto CHARGE_PIN = GPIO_NUM_32;
constexpr auto BATT_ADC_PIN = GPIO_NUM_35;

/* EEPROM */
constexpr int WIFI_STATE_ADDRESS = 0x00;

/* TFT screen */
#define SEG7_BACKGROUND 0x0821
