#include "pros/rtos.hpp"
#include "pros/adi.hpp"

#ifndef LEDS_HPP
#define LEDS_HPP

class Leds : public pros::ADILed
{
public:
    Leds(std::uint8_t adi_port, std::uint32_t length);
    void flashing_seizure(void *param);
    
};

#endif