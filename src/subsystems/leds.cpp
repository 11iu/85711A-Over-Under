#include "leds.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"

Leds::Leds(std::uint8_t adi_port, std::uint32_t length) : pros::ADILed(adi_port, length)
{  
};

void Leds::flashing_seizure(void *param)
{
    while (true)
    {
        set_all(0xFFFFFF);
        pros::delay(200);
        clear_all();
        pros::delay(200);
    }
}