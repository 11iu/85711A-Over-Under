#include "../include/main.h"
#include "autons.hpp"
#include "constants.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//  chassis motors
pros::Motor lF(LEFT_FRONT, pros::E_MOTOR_GEARSET_06, true); // reverse as needed
pros::Motor lM(LEFT_MIDDLE, pros::E_MOTOR_GEARSET_06, false);
pros::Motor lB(LEFT_BACK, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rF(RIGHT_FRONT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rM(RIGHT_MIDDLE, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rB(RIGHT_BACK, pros::E_MOTOR_GEARSET_06, false);

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB});  // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// Inertial Sensor
pros::Imu imu(IMU);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left motor group
    &rightMotors,               // right motor group
    10,                         // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    360,                        // drivetrain rpm is 360
    2                           // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings
    linearController(10,  // proportional gain (kP)
                     0,   // integral gain (kI)
                     3,   // derivative gain (kD)
                     3,   // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     20   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(2,   // proportional gain (kP)
                      0,   // integral gain (kI)
                      10,  // derivative gain (kD)
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

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors);

void arcade_drive(bool flipDrive = false)
{
  // get joystick positions
  int leftY = lemlib::defaultDriveCurve(
      controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2);
  int rightX = lemlib::defaultDriveCurve(
      controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2);
  if (flipDrive)
    leftY *= -1;
  // move the chassis with arcade drive
  chassis.arcade(leftY, rightX);
}

void initialize()
{
  pros::lcd::initialize(); // initialize brain screen
  pros::delay(
      500); // Stop the user from doing anything while legacy ports configure.

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
      Auton("Test autonomous", autoTest),
      Auton("AWP\n\nStart for autoAttack on defense side with triball", awp),
      Auton("Auto Attack Blue\n\nStart in farthest full starting tile, facing the "
            "center of the field",
            autoAttackBlue),
      Auton("Auto Attack Red\n\nStart in farthest full starting tile, facing the "
            "center of the field",
            autoAttackRed),
      Auton("Auto Defense Blue\n\nStart in closest tile, touching the match load "
            "area, no triball",
            autoDefenseBlue),
      Auton("Auto Defense Red\n\nStart in closest tile, touching the match load "
            "area, no triball",
            autoDefenseRed),
      Auton("Auto Skills\n\nSetup like autoDefense, with triballs galore",
            autoSkills),
  });

  ez::as::initialize(); // initialize auton selector

  chassis.calibrate(); // calibrate sensors

  pros::ADIDigitalOut wings_initializer(WINGS, LOW);

  pros::Motor inake_initializer(INTAKE, pros::E_MOTOR_GEARSET_18, false,
                                pros ::E_MOTOR_ENCODER_DEGREES);
  pros::Motor cata_initializer(CATA, pros::E_MOTOR_GEARSET_36, true);
  inake_initializer.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  cata_initializer.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); // TODO - test

  // thread to for brain screen and position logging
  /*
  pros::Task screenTask([&]()
                        {
    lemlib::Pose pose(0, 0, 0);
    while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f",  chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f",  chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f",  chassis.getPose().theta); // heading
        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);
    } });
  */
}

void autonomous()
{
  ez::as::auton_selector
      .call_selected_auton(); // Calls selected auton from autonomous selector.
}

void opcontrol()
{
  bool flipDrive = false;
  bool wingState = LOW; // wings wingState

  int delayWings = 0;
  int delayFlip = 0;

  while (true)
  {
    arcade_drive(flipDrive);

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

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      cata = CATAMAXVOLTAGE; // fire and continuous fire
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
    arcade_drive();

    pros::delay(10);
  }
}
