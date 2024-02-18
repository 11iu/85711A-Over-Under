#include "main.h"
#include "constants.hpp"
#include "display/lv_misc/lv_color.h"
#include "field.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

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
pros::MotorGroup rightMotors({rF, rM,
                              rB}); // Creates a motor group with forwards port

// Inertial Sensor
pros::Imu imu(IMU_PORT);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left motor group
    &rightMotors,               // right motor group
    12,                         // 12 inch track width (left to right wheels)
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    360,                        // drivetrain rpm is 360
    8                           // chase power is 2. If we had traction wheels, it would have been 8
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
pros::ADIDigitalOut vertWings(VERT_WINGS, LOW);

pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEARSET_18, false,
                   pros ::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata(CATA_PORT, pros::E_MOTOR_GEARSET_36, true);

// ASSET(autoSkillsPath_txt); // for path

// poses only defined for red side, as blue is the same but flipped!

// starts at opposite of close side facing towards goal, pushes triball into the
// goal, and sets up for match load
void autoCloseOpposite()
{
  chassis.setPose(closeOppStart.x, closeOppStart.y, closeOppStart.angle);
  chassis.moveToPose(blueGoalLeftSide.x + 5, blueGoalLeftSide.y, 90, 2000, {.minSpeed = 100}, false); // push into the goal
  intake = 127;
  pros::delay(500);
  chassis.moveToPose(closeOppEnd.x, closeOppEnd.y, closeOppEnd.angle, 2000, {.forwards = false, .maxSpeed = 80}, false);
  intake = 0;
}

// starts at close side facing towards goal, pushes triball into the goal, and
// sets up for match load
void autoClose()
{
  chassis.setPose(closeStart.x, closeStart.y, closeStart.angle);
  chassis.moveToPose(blueGoalRightSide.x - 5, blueGoalRightSide.y, -90, 2000, {.minSpeed = 100}, false);
  intake = 127;
  pros::delay(500);
  chassis.moveToPose(closeEnd.x, closeEnd.y, closeOppEnd.angle, 2000, {.forwards = false, .maxSpeed = 80}, false);
  intake = 0;
}

// start in farthest full starting tile, facing the center of the field
// starts at upper
void autoFar()
{
  chassis.setPose(farStart.x, farStart.y, farStart.angle);
  chassis.moveToPose(fieldX / 2, farStart.y, farStart.angle, 4000, {.minSpeed = 80}, false); // Moves to in front of goal
  chassis.moveToPose(fieldX / 2, farStart.y, 0, 4000, {}, false); // turn to face goal
  intake = 127;
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - tile / 2, 0, 4000, {.minSpeed = 80}, false); // Shoves preload in
  intake = 0;
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 4000, {.forwards = false}, false); // back out
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 4000, {}, false); // turn towards center

  // grab upper triball and score
  intake = -127;
  chassis.moveToPose(blueCenterLowerTriball.x - 3, blueCenterLowerTriball.y + 4, 180, 2000, {}, false); // move into the triball
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 2000, {.forwards = false}, false); // face goal
  intake = 127;
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - 4, 0, 4000, {.minSpeed = 100}, false); // score
  intake = 0;

  // reset for teleop
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 4000,
                     {.forwards = false}, false); // back out

  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 3000, {},
                     false); // turn towards center

  chassis.moveToPose(fieldX / 2.0, redGoalCenter.y - 2, 180, 750,
                     {.forwards = false}, false);
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 500, {},
                     false); // turn towards center
}

// starts in far side facing the goal, completes awp
// Created for red side far
void autoFarAWP() {}

// start the same as autoClose
void autoSkills()
{

  chassis.setPose(closeStart.x, closeStart.y, closeStart.angle);
  chassis.moveToPose(blueGoalRightSide.x - 5, blueGoalRightSide.y, -90, 2000, {.minSpeed = 100}, false);
  intake = 127;
  pros::delay(500);
  chassis.moveToPose(closeEnd.x - 4, closeEnd.y, closeOppEnd.angle, 2000,
                     {.forwards = false, .maxSpeed = 80},
                     false); // small change
  intake = 0;
  chassis.tank(0, -30); // push back to prevent cata momentum pushing forward
  cata = CATAMAXVOLTAGE;
  pros::delay(30000); // correct timing
  cata = 0;
  chassis.tank(0, 0);

  // recalibrate our position by ramming backwards into the angled corner bar
  chassis.moveToPose(fieldX - tile * 1.2, tile * 1.2, -45, 4000, {.minSpeed = 80}, false);
  chassis.moveToPose(fieldX - tile * 0.6, tile * 0.6, -45, 2000, {.forwards = false, .minSpeed = 80}, false);
  chassis.setPose(fieldX - tile * 0.9, tile * 0.9, -45);

  // go to the other side and push into right side of goal
  chassis.moveToPose(fieldX - tile / 2.0, tile + 5, 0, 4000, {.minSpeed = 80}, false);
  chassis.moveToPose(fieldX - tile / 2.0, fieldY - tile * 1.5, 0, 4000, {.minSpeed = 80}, false);
  chassis.moveToPose(fieldX - tile / 2.0, fieldY - tile * 1.5, -60, 4000, {.minSpeed = 80}, false);
  chassis.moveToPose(redGoalRightSide.x - 6, redGoalRightSide.y, -60, 4000, {.minSpeed = 100}, false);
  chassis.moveToPose(fieldX - tile, fieldY - tile, -160, 4000, {.forwards = false}, false);

  // ramming into the center from the right, straight on, then left
  chassis.moveToPose(fieldX - tile * 2, fieldY / 2.0 + 12, -160, 2000, {.minSpeed = 100}, false); // line up to right of goal
  chassis.moveToPose(fieldX - tile * 2, fieldY / 2.0 + 12, -20, 2000, {}, false); // turn towards goal
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, -20, 2000, {.minSpeed = 100}, false);
  pros::delay(200);

  wings.set_value(LOW);
  chassis.moveToPose(fieldX / 2, fieldY / 2.0 + 12, 0, 2000, {.forwards = false}, false); // line up in front of the goal
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, 0, 2000, {.minSpeed = 100}, false);
  pros::delay(200);

  wings.set_value(LOW);
  chassis.moveToPose(tile * 2, fieldY / 2.0 + 12, 20, 2000, {.forwards = false}, false); // line up to the left of goal
  wings.set_value(HIGH);
  pros::delay(200);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, 20, 2000, {.minSpeed = 100}, false);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - tile, 20, 2000, {.minSpeed = 100}, false);
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
      logDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2);

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

void autoDisabled()
{
  // do nothing
}

struct Auto
{
  std::string name;
  void (*function)(void);
};

Auto autoFarAuton{"Auto Far", autoFar};
Auto autoCloseAuton{"Auto Close", autoClose};
Auto autoSkillsAuton{"Auto Skills", autoSkills};
Auto autoDisabledAuton{"Disabled", autoDisabled};

std::vector<Auto> autos = {autoFarAuton, autoCloseAuton, autoSkillsAuton,
                           autoDisabledAuton};
int currentAuto = 2;

void initialize()
{
  pros::delay(500); // Stop the user from doing anything while
                    // legacy ports configure.
  pros::lcd::initialize();
  chassis.calibrate();
}

void pgUp()
{
  currentAuto = currentAuto + 1;
  if (currentAuto > autos.size() - 1)
    currentAuto = 0;
  pros::lcd::print(0, "%s", autos[currentAuto].name);
}
void pgDown()
{
  currentAuto = currentAuto - 1;
  if (currentAuto < 0)
    currentAuto = autos.size() - 1;
  pros::lcd::print(0, "%s", autos[currentAuto].name);
}

void competition_initialize()
{
  currentAuto = 0;
  pros::ADIDigitalIn limit_left('b');
  pros::ADIDigitalIn limit_right('c');
  pros::lcd::register_btn0_cb(pgDown);
  pros::lcd::register_btn2_cb(pgUp);
  pros::lcd::print(0, "%s", autos[currentAuto].name);

  while (true)
  {
    if (limit_left.get_value())
    {
      pgUp();
      pros::delay(500);
    }
    else if (limit_right.get_value())
    {
      pgDown();
      pros::delay(500);
    }
    pros::delay(20);
  }
}

void autonomous() { ((void (*)())autos[currentAuto].function)(); }

void opcontrol()
{
  bool flipDrive = false;
  bool wingState = LOW;     // wings wingState
  bool vertWingState = LOW; // vertical wings wingState
  bool cataFire = false;    // toggle for catapult

  int delayVertWing = 0;
  int delayWings = 0;
  int delayCata = 0;
  int delayFlip = 0;
  bool moving = true;

  while (true)
  {
    int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    bool moving = abs(forward) > 10 || abs(turn) > 10;

    int leftY = logDrive(forward, 2);
    int rightX = logDrive(turn, 3);

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

    // wing
    if (delayVertWing)
    {
      delayVertWing--;
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      vertWingState = !vertWingState;
      vertWings.set_value(vertWingState);
      delayVertWing = 40;
    }

    // cata toggle
    if (!delayCata)
    {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
      {
        cataFire = !cataFire;
        delayCata = 40;
      }
    }
    else
    {
      delayCata--;
    }

    if (cataFire)
    {
      cata = CATAMAXVOLTAGE; // continuous fire
      if (!moving &&
          autos[currentAuto].name ==
              autoSkillsAuton.name)
      { // drive backwards if we are in skills, so
        // we can be more accurate.
        chassis.tank(0, -30);
      }
    }
    else
    {
      cata.brake(); // coast up
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

    pros::delay(20);
  }
}