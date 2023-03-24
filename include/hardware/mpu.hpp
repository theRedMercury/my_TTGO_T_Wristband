#pragma once

#include "hal.hpp"
#include "watch/watch_abs.hpp"

#include <SparkFunMPU9250-DMP.h>

class mpu_manager final : public watch_abs
{
public:
    using watch_abs::watch_abs;
    void setup() override;
    void update();
    void light_sleep();
    void deep_sleep();

    auto get_heading() const -> int16_t;

    auto get_temp() const -> float;
    auto get_steps() const -> ulong;
    auto get_step_time() const -> ulong;

    void reset_step();

    inline float get_accelX() const
    {
        return accelX;
    }
    inline float get_accelY() const
    {
        return accelY;
    }
    inline float get_accelZ() const
    {
        return accelZ;
    }
    inline float get_gyroX() const
    {
        return gyroX;
    }
    inline float get_gyroY() const
    {
        return gyroY;
    }
    inline float get_gyroZ() const
    {
        return gyroZ;
    }
    inline float get_magX() const
    {
        return magX;
    }
    inline float get_magY() const
    {
        return magY;
    }
    inline float get_magZ() const
    {
        return magZ;
    }

    inline float get_roll() const
    {
        return roll;
    }
    inline float get_pitch() const
    {
        return pitch;
    }
    inline float get_yaw() const
    {
        return yaw;
    }

private:
    MPU9250_DMP _imu;

    float accelX = 0.F;
    float accelY = 0.F;
    float accelZ = 0.F;
    float gyroX = 0.F;
    float gyroY = 0.F;
    float gyroZ = 0.F;
    float magX = 0.F;
    float magY = 0.F;
    float magZ = 0.F;
    float roll = 0.F;
    float pitch = 0.F;
    float yaw = 0.F;

    long temp;
    unsigned long time;
    int16_t heading = 0;
};
