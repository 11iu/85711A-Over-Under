#include "leds.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

Leds::Leds(std::uint8_t adi_port, std::uint32_t length) : pros::ADILed(adi_port, length){};

void Leds::flashing_seizure(void *param)
{
    while ((pros::c::competition_get_status() & COMPETITION_AUTONOMOUS) != 0)
    {
        set_all(0xFFFFFF);
        pros::delay(200);
        clear_all();
        pros::delay(200);
    }
}

void Leds::sequential_individual(void *param)
{
    uint32_t start = pros::millis();
    clear_all();

    while(true) {
        uint32_t current = pros::millis();
        if (current - start > 60000)
        {
            clear_all();
            start = current;
        }
        else
        {
            _buffer[(current - start) / 1000] = 0x0000FF;
            update();
        }
    }
}