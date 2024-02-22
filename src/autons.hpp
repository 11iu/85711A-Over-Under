#ifndef AUTONS_HPP
#define AUTONS_HPP

#include "lemlib/api.hpp"
#include "constants.hpp"
#include "pros/distance.hpp"

class Autons {
private:
    lemlib::Chassis &chassis;
    pros::ADIDigitalOut &wings;
    pros::ADIDigitalOut &vertWings;

    pros::Motor &intake;
    pros::Motor &cata;

    pros::ADIUltrasonic &rightSonic;
    pros::Distance &distanceBack;
    pros::ADIUltrasonic &intakeSonic;

public:
    Autons(lemlib::Chassis &chassis, pros::ADIDigitalOut &wings, pros::ADIDigitalOut &vertWings, pros::Motor &intake, pros::Motor &cata, pros::ADIUltrasonic &rightSonic, pros::Distance &distanceBack, pros::ADIUltrasonic &intakeSonic);
    
    // Function declarations
    std::pair<float, float> localizeRobot();
    bool hasTriball();
    void fireCata();
    void toggleWings();
    void toggleVertWings();
    void setWings(bool state);
    void setVertWings(bool state);
    void autoCloseOpposite();
    void autoClose();
    void autoFar();
    void autoFarAWP();
    void autoSkills();
    void autoDisabled();
};

#endif // AUTONS_HPP
