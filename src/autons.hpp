#pragma once

#include "constants.hpp"
#include "main.h"

// create objects for hardware, to be used in main and autons
extern lemlib::Chassis chassis;
extern pros::ADIDigitalOut wings(WINGS, LOW);
extern pros::Motor intake(INTAKE, pros::E_MOTOR_GEARSET_18, false,
                          pros ::E_MOTOR_ENCODER_DEGREES);
extern pros::Motor cata(CATA, pros::E_MOTOR_GEARSET_36, true);

void autoTest();
void awp();
void autoAttackBlue();
void autoAttackRed();
void autoDefenseBlue();
void autoDefenseRed();
void autoSkills();
