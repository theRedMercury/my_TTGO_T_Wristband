#include "hardware/mpu.hpp"
#include "hardware/eeprom.hpp"
#include "tools.hpp"

RTC_DATA_ATTR ulong steps = 0;
RTC_DATA_ATTR ulong step_time = 0;

void mpu_manager::setup()
{
    DEBUG_PRINTLN("mpu_manager setup");
    if (_imu.begin() != INV_SUCCESS)
    {
        while (1)
        {
            DEBUG_PRINTLN("Unable to communicate with MPU-9250");
            DEBUG_PRINTLN("Check connections, and try again.");
            DEBUG_PRINTLN();
            delay(SEC_IN_MS * 5);
        }
    }

    ulong s = 0;
    _imu.dmpGetPedometerSteps(s);
    if (s > 0 && s < 0xFFFFFF)
    {
        DEBUG_PRINTLN("steps : " + String(s));
        steps = s;
    } else
    {
        const ulong ss = epprom_mem::get_steps_mem();
        if (ss > 0 && ss < 0xFFFFFF)
        {
            steps = ss;
        } else
        {
            steps = 0;
        }
    }
    DEBUG_PRINTLN("steps : " + String(steps));

    s = 0;
    _imu.dmpGetPedometerTime(s);
    if (s > 0 && s < 0xFFFFFF)
    {
        step_time = s;
    } else
    {
        step_time = epprom_mem::get_steps_time_mem();
    }

    _imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    _imu.setGyroFSR(2000); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    _imu.setAccelFSR(2); // Set accel to +/-2g
    // Note: the MPU-9250's magnetometer FSR is set at
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    _imu.setLPF(5); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    _imu.setSampleRate(10); // Set sample rate to 10Hz

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    _imu.setCompassSampleRate(10);

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    // _imu.setSampleRate(100); // Set sample rate to 100Hz

    _imu.selfTest();

    // Use configureFifo to set which sensors should be stored
    // in the buffer.
    // Parameter to this function can be: INV_XYZ_GYRO,
    // INV_XYZ_ACCEL, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    //_imu.configureFifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    _imu.dmpBegin(DMP_FEATURE_PEDOMETER | DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO,
                  10);
    _imu.dmpSetPedometerSteps(steps);
    _imu.dmpSetPedometerTime(step_time);
}

void mpu_manager::calibrate()
{
    _imu.calibrate();
    setup();
}

void mpu_manager::update()
{
    if (_imu.dataReady() && _imu.fifoAvailable() && _imu.dmpUpdateFifo() == INV_SUCCESS &&
        _imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS | UPDATE_TEMP) == INV_SUCCESS)
    {
        // Call update() to update the imu objects sensor data.
        // You can specify which sensors to update by combining
        // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
        // UPDATE_TEMPERATURE.
        // (The update function defaults to accel, gyro, compass,
        //  so you don't have to specify these values.)

        accelX = _imu.calcAccel(_imu.ax);
        accelY = _imu.calcAccel(_imu.ay);
        accelZ = _imu.calcAccel(_imu.az);

        gyroX = _imu.calcGyro(_imu.gx);
        gyroY = _imu.calcGyro(_imu.gy);
        gyroZ = _imu.calcGyro(_imu.gz);

        magX = _imu.calcMag(_imu.mx);
        magY = _imu.calcMag(_imu.my);
        magZ = _imu.calcMag(_imu.mz);

        // computeEulerAngles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        _imu.computeEulerAngles(true);
        roll = _imu.roll;
        pitch = _imu.pitch;
        yaw = _imu.yaw;
        yaw = 360.F - yaw;

        temp = static_cast<float>(_imu.temperature);

        _imu.computeCompassHeading();

        // DEBUG_PRINTLN("R/P/Y: " + String(roll) + ", " + String(pitch) + ", " + String(yaw));
        // DEBUG_PRINTLN("heading: " + String(heading));

        // float q0 = _imu.calcQuat(_imu.qw);
        // float q1 = _imu.calcQuat(_imu.qx);
        // float q2 = _imu.calcQuat(_imu.qy);
        // float q3 = _imu.calcQuat(_imu.qz);
    }

    _imu.dmpGetPedometerSteps(steps);
    _imu.dmpGetPedometerTime(step_time);
}

auto mpu_manager::get_heading() const -> int16_t
{
    return static_cast<int16_t>(_imu.heading);
}

auto mpu_manager::get_temp() const -> float
{
    return temp;
}

auto mpu_manager::get_steps() const -> ulong
{
    return steps;
}

/* return ms */
auto mpu_manager::get_step_time() const -> ulong
{
    return step_time;
}

void mpu_manager::reset_step()
{
    DEBUG_PRINTLN("reset_step()");
    steps = 0;
    step_time = 0;
    _imu.dmpSetPedometerSteps(steps);
    _imu.dmpSetPedometerTime(step_time);
}

void mpu_manager::light_sleep()
{
    _imu.setSensors(0);
    /*if (!_imu.light_sleep())
    {
        DEBUG_PRINTLN("FUCK");
    }*/
}

void mpu_manager::deep_sleep()
{
    //_imu.deep_sleep();
}
