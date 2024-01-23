#pragma once

#include "main.h"
#include "constants.hpp"

// create objects for hardware, to be used in main and autons
extern lemlib::Chassis chassis;
extern pros::ADIDigitalOut wings(WINGS);
extern pros::Motor intake(INTAKE);
extern pros::Motor cata(CATA);

void autoTest();
void awp();
void autoAttackBlue();
void autoAttackRed();
void autoDefenseBlue();
void autoDefenseRed();
void autoSkills();
