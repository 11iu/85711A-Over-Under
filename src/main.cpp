#include "main.h"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/auton_selector.hpp"
#include "EZ-Template/sdcard.hpp"
#include "constants.hpp"
#include "field.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

//  chassis motors
pros::Motor lF(LEFT_FRONT_LOWER_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor lM(LEFT_FRONT_UPPER_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor lB(LEFT_BACK_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rF(RIGHT_FRONT_LOWER_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rM(RIGHT_FRONT_UPPER_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rB(RIGHT_BACK_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::MotorGroup leftMotors({lF, lM,
                             lB}); // Creates a motor group with forwards ports
                                   // 1 & 3 and reversed port 2
pros::MotorGroup rightMotors({rF, rM,
                              rB}); // Creates a motor group with forwards port
                                    // 4 and reversed ports 4 & 6

// Inertial Sensor
pros::Imu imu(IMU_PORT);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left motor group
    &rightMotors,               // right motor group
    12,                         // 12 inch track width (left to right wheels)
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    360,                        // drivetrain rpm is 360
    2                           // chase power is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings
    linearController(20,  // proportional gain (kP)
                     0,   // integral gain (kI)
                     4,   // derivative gain (kD)
                     3,   // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     40   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(3,   // proportional gain (kP)
                      0,   // integral gain (kI)
                      15,  // derivative gain (kD) 10 works
                      3,   // anti windup
                      1,   // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0    // maximum acceleration (slew)
    );

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't
// pass vertical tracking wheels
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors);
pros::ADIDigitalOut wings(WINGS, LOW);
pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEARSET_18, false,
                   pros ::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata(CATA_PORT, pros::E_MOTOR_GEARSET_36, true);

void autoSkillsDriverAKAOppositeOfAutoClose()
{
  chassis.setPose(driverAutoStart.x, driverAutoStart.y, driverAutoStartHeading);
  chassis.moveToPose(driverAutoMid.x, driverAutoMid.y, driverAutoMidHeading,
                     2000, {.minSpeed = 80}, false);
  chassis.moveToPose(driverAutoMid2.x, driverAutoMid2.y, driverAutoMid2Heading,
                     2000, {.forwards = false, .chasePower = 1, .minSpeed = 80},
                     false);
  chassis.moveToPose(driverAutoEnd.x, driverAutoEnd.y, driverAutoEndHeading,
                     2000, {.maxSpeed = 80}, false);
  wings.set_value(HIGH);
}

void autoClose()
{
  chassis.setPose(fieldX - driverAutoStart.x, driverAutoStart.y, -135);
  chassis.moveToPose(fieldX - driverAutoMid.x, driverAutoMid.y, -driverAutoMidHeading,
                     2000, {.minSpeed = 80}, false);
  chassis.moveToPose(fieldX - driverAutoMid2.x, driverAutoMid2.y, -driverAutoMid2Heading,
                     2000, {.forwards = false, .chasePower = 1, .minSpeed = 80},
                     false);
  chassis.moveToPose(fieldX - driverAutoEnd.x, driverAutoEnd.y, -driverAutoEndHeading,
                     2000, {.maxSpeed = 80}, false);
  wings.set_value(HIGH);
}

// start in farthest full starting tile, facing the center of the field
// starts at upper
void autoFarRed()
{
  chassis.setPose(redStartUpper.x, redStartUpper.y, redStartUpperHeading);
  chassis.moveToPose(fieldX / 2, redStartUpper.y, redStartUpperHeading,
                     4000);                                 // Moves to face the goal
  chassis.moveToPose(fieldX / 2, redStartUpper.y, 0, 4000); // turn
  pros::delay(500);
  intake = 127; // score preload
  pros::delay(1000);
  chassis.moveToPose(fieldX / 2, redStartUpper.y + tile, 0, 4000, {.minSpeed = 80}); // Shoves the triball in
  intake = 0;
  chassis.moveToPose(fieldX / 2, redStartUpper.y, 0, 4000, {.forwards = false});     // back up and ram
  chassis.moveToPose(fieldX / 2, redStartUpper.y + tile, 0, 4000, {.minSpeed = 80}); // ram the triball in
  chassis.moveToPose(fieldX / 2, redStartUpper.y - tile / 2, 180, 4000,
                     {.forwards = false}); // Move backwards and get ready for teleop
}

// shoots all triballs and scores with wings
// start in lower left corner between goal and corner facing the goal
void autoSkillsRed()
{
  autoSkillsDriverAKAOppositeOfAutoClose();
  // cata = CATAMAXVOLTAGE;
  // pros::delay(25000); // wait 25 sec
  // cata = 0;
  wings.set_value(LOW);

  // go to the other side and push into left side
  chassis.moveToPose(tile / 2.0, tile * 1.5, 0, 2000, {.minSpeed = 80}, false);
  chassis.moveToPose(tile / 2.0, fieldY - tile, 60, 4000, {.maxSpeed = 80},
                     false);
  chassis.moveToPose(redGoalLeftSide.x, redGoalLeftSide.y, 60, 4000, {}, false);
  chassis.moveToPose(tile, fieldY - tile, 135, 2000, {.forwards = false},
                     false);

  // ramming into the center
  chassis.moveToPose(tile * 1.5, fieldY / 2 + 4, 30, 4000, {}, false);
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, 30, 4000, {}, false);
  wings.set_value(LOW);
  chassis.moveToPose(blueCenterLowerTriball.x, blueCenterLowerTriball.y - 4, 0,
                     4000, {.forwards = false},
                     false); // line up in front of the goal
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, 0, 4000, {}, false);
  wings.set_value(LOW);
  chassis.moveToPose(fieldX - tile * 1.5, fieldY / 2 + 4, -30, 4000,
                     {.forwards = false}, false);
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, -30, 4000, {}, false);
}

double logDrive(double v, double pow)
{
  if (v > 0)
  {
    return (std::pow(std::abs(v), pow) / std::pow(127, pow)) * 127;
  }
  else
  {
    return -1 * (std::pow(std::abs(v), pow) / std::pow(127, pow)) * 127;
  }
}

// do not use curvature drive it is buggy af
void arcade_drive(bool flipDrive = false)
{
  // if () // TODO: add deadzone

  // int leftY = pow(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) /
  // 127.0, 3) * 127; int rightX =
  // pow(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, 3) *
  // 127;

  int leftY = logDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2);
  int rightX =
      logDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 3);

  // turbo mode is right bottom trigger
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
  {
    leftY = leftY * TURBO_FORWARD;
    rightX = rightX * TURBO_TURN;
  }
  else
  {
    leftY = leftY * REGULAR_FORWARD;
    rightX = rightX * REGULAR_TURN;
  }

  if (flipDrive)
    leftY *= -1;
  // move the chassis with arcade drive
  chassis.arcade(leftY, rightX);
}

// TODO - need to test this
void set_braking(bool brakeCoast = true)
{
  if (brakeCoast)
  {
    leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  }
  else
  {
    leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  }
}

void auto_disabled()
{
  // do nothing
}

Auton autoFarAuton("Auto Far", autoFarRed);
Auton autoCloseAuton("Auto Close", autoClose);
Auton autoSkillsAuton("Auto Skills", autoSkillsRed);
Auton autoDisabled("Disabled", auto_disabled);

void initialize()
{
  pros::delay(500); // Stop the user from doing anything while
                    // legacy ports configure.

  ez::as::auton_selector.add_autons({autoCloseAuton, autoFarAuton,
                                     autoSkillsAuton, autoDisabled});
  ez::as::initialize();

  chassis.calibrate();
}

void autonomous()
{
  ez::as::auton_selector.call_selected_auton();
}

void opcontrol()
{
  int cataHeadStart = 0;
  if (ez::as::auton_selector.Autons[ez::as::auton_selector.current_auton_page]
          .Name == autoSkillsAuton.Name)
  {
    autoSkillsDriverAKAOppositeOfAutoClose();
    cataHeadStart = 200;
  }
  bool flipDrive = false;
  bool wingState = LOW; // wings wingState

  int delayWings = 0;
  int delayFlip = 0;

  while (true)
  {

    // wings
    if (delayWings)
    {
      delayWings--;
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
      wingState = !wingState;
      wings.set_value(wingState);
      delayWings = 40;
    }

    // cata
    // cataDown = limit_switch.get_value();
    // cataDown = pot.get_value() > CATA_THRESHOLD;  // we are using the limit
    // switch

    if (cataHeadStart > 0)
    {
      cata = CATAMAXVOLTAGE;
      cataHeadStart--;
    }
    else
    {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
      {
        cata = CATAMAXVOLTAGE; // fire and continuous fire
      }
      else
      {
        cata.brake(); // coast up
      }
    }

    // intake
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
      intake = 127;
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      intake = -127;
    }
    else
    {
      intake.brake();
    }

    // filpDrive
    if (!delayFlip)
    {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
      {
        flipDrive = !flipDrive;
        delayFlip = 40;
      }
    }
    else
    {
      delayFlip--;
    }

    arcade_drive(flipDrive);
    pros::delay(20);
  }
}