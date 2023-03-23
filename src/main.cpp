#include <Arduino.h>

#include "watch/watch.hpp"

static watch_manager watch;

void setup()
{

#ifdef DEBUG_MODE
    DEBUG_PRINTER.begin(115200);
    DEBUG_PRINTLN("##########");
    DEBUG_PRINTLN("#  INIT  #");
    DEBUG_PRINTLN("##########");
#endif

    watch.setup();
}

void loop()
{
    delay(2000);
}
