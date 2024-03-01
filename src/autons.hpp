#ifndef AUTONS_HPP
#define AUTONS_HPP

#include "constants.hpp"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"

class Autons {
private:
  lemlib::Chassis &chassis;
  pros::ADIDigitalOut &wings;
  pros::ADIDigitalOut &vertWings;

  pros::Motor &intake;
  pros::Motor &cata;

  pros::Distance &distRight;
  pros::ADIUltrasonic &distBack;
  pros::ADIUltrasonic &distIntake;

public:
  Autons(lemlib::Chassis &chassis, pros::ADIDigitalOut &wings,
         pros::ADIDigitalOut &vertWings, pros::Motor &intake, pros::Motor &cata,
         pros::Distance &distRight, pros::ADIUltrasonic &distBack,
         pros::ADIUltrasonic &distIntake);

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
//   std::pair<Pose2d, doublse> calculatePositionAngle();
};

#endif // AUTONS_HPP
