#pragma once
#include <Arduino.h>
#include <stdint.h>

class watch_manager;

class watch_abs
{
public:
    watch_abs(watch_manager& watch);
    watch_abs() = delete;
    watch_abs(const watch_abs&) = delete;
    watch_abs(watch_abs&&) = delete;
    watch_abs& operator=(const watch_abs&) = delete;
    watch_abs& operator=(watch_abs&&) = delete;

    virtual inline void setup() = 0;

protected:
    watch_manager* _watch;
};
