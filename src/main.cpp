#include "main.h"
#include "constants.hpp"
#include "field.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"

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
    2 // chase power is 2. If we had traction wheels, it would have been 8
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

// ASSET(autoSkillsPath_txt); // for path

// poses only defined for red side, as blue is the same but flipped!

// starts at opposite of close side facing towards goal, pushes triball into the
// goal, and sets up for match load
void autoCloseOpposite() {
  chassis.setPose(closeOppStart.x, closeOppStart.y, closeOppStart.angle);
  chassis.moveToPose(blueGoalLeftSide.x + 5, blueGoalLeftSide.y, 90, 2000,
                     {.minSpeed = 100}, false); // push into the goal
  intake = 127;
  chassis.moveToPose(closeOppEnd.x, closeOppEnd.y, closeOppEnd.angle, 2000,
                     {.forwards = false, .maxSpeed = 80}, false);
  intake = 0;
  pros::delay(400);
  wings.set_value(HIGH);
}

// starts at close side facing towards goal, pushes triball into the goal, and
// sets up for match load
void autoClose() {
  chassis.setPose(closeStart.x, closeStart.y, closeStart.angle);
  chassis.moveToPose(blueGoalRightSide.x - 5, blueGoalRightSide.y, -90, 4000,
                     {.minSpeed = 100}, false);
  intake = 127;
  chassis.moveToPose(closeEnd.x, closeEnd.y, closeOppEnd.angle, 2000,
                     {.forwards = false, .maxSpeed = 80}, false);
  intake = 0;
  pros::delay(400);
  wings.set_value(HIGH);
}

// start in farthest full starting tile, facing the center of the field
// starts at upper
void autoFar() {
  chassis.setPose(farStart.x, farStart.y, farStart.angle);
  chassis.moveToPose(fieldX / 2, farStart.y, farStart.angle, 4000,
                     {.minSpeed = 100}, false); // Moves to in front of goal
  chassis.moveToPose(fieldX / 2, farStart.y, 0, 2000, {},
                     false); // turn to face goal
  intake = 127;
  chassis.moveToPose(fieldX / 2, farStart.y + tile, 0, 2000, {.minSpeed = 80},
                     false); // Shoves preload in
  intake = 0;
  // chassis.moveToPose(fieldX / 2, redStartUpper.y, 0, 2000, {.forwards =
  // false, .minSpeed = 100}, false); // back up and ram
  // chassis.moveToPose(fieldX / 2, redStartUpper.y + tile, 0, 4000, {.minSpeed
  // = 100}, false);         // ram the triball in
  intake = -127;
  chassis.moveToPose(blueCenterLowerTriball.x - 6, tile * 2, 0, 2000,
                     {.forwards = false, .minSpeed = 80},
                     false); // Move backwards to pick up another triball
  chassis.moveToPose(blueCenterLowerTriball.x, blueCenterLowerTriball.y - 6,
                     180, 2000, {},
                     false); // Move backwards to pick up another triball
  chassis.moveToPose(fieldX / 2, farStart.y + tile, 0, 2000, {.minSpeed = 80},
                     false); // Shoves the triball in
  intake = 127;
  pros::delay(500);
  intake = 0;
  chassis.moveToPose(fieldX / 2, farStart.y, 180, 2000,
                     {.forwards = false, .minSpeed = 100}, false); // back up
}

// starts in far side facing the goal, completes awp
// Created for red side far
void autoFarAWP() {}

// shoots all triballs and scores with wings
// start in lower right corner between goal and corner facing the goal
void autoSkills() {

  autoCloseOpposite();
  // cata = CATAMAXVOLTAGE;
  // pros::delay(25000); // wait 25 sec
  // cata = 0;
  wings.set_value(LOW);

  // chassis.follow(autoSkillsPath_txt, 15, 6000, true);

  // go to the other side and push into left side
  chassis.moveToPose(tile / 2.0, tile + 5, 0, 2000, {.minSpeed = 80}, false);
  chassis.moveToPose(tile / 2.0, fieldY - tile, 0, 4000, {.maxSpeed = 80},
                     false);
  chassis.moveToPose(tile / 2.0, fieldY - tile, 60, 4000, {.maxSpeed = 80},
                     false);
  chassis.moveToPose(redGoalLeftSide.x, redGoalLeftSide.y, 60, 4000, {}, false);
  chassis.moveToPose(tile, fieldY - tile, 135, 2000, {.forwards = false},
                     false);

  // ramming into the center
  chassis.moveToPose(tile * 1.5, fieldY / 2.0 + 8, 30, 4000, {}, false);
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, 30, 4000, {}, false);
  wings.set_value(LOW);
  chassis.moveToPose(blueCenterLowerTriball.x, fieldY / 2.0 + 12, 0, 4000,
                     {.forwards = false},
                     false); // line up in front of the goal
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, 0, 4000, {}, false);
  wings.set_value(LOW);
  chassis.moveToPose(fieldX - tile * 1.5, fieldY / 2.0 + 12, -30, 4000,
                     {.forwards = false}, false);
  wings.set_value(HIGH);
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, -30, 4000, {}, false);
}

double logDrive(double v, double pow) {
  if (v > 0) {
    return (std::pow(std::abs(v), pow) / std::pow(127, pow)) * 127;
  } else {
    return -1 * (std::pow(std::abs(v), pow) / std::pow(127, pow)) * 127;
  }
}

// do not use curvature drive it is buggy af
void arcade_drive(bool flipDrive = false) {
  // if () // TODO: add deadzone

  // int leftY = pow(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) /
  // 127.0, 3) * 127; int rightX =
  // pow(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, 3) *
  // 127;

  int leftY = logDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2);
  int rightX =
      logDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 3);

  // turbo mode is right bottom trigger
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    leftY = leftY * TURBO_FORWARD;
    rightX = rightX * TURBO_TURN;
  } else {
    leftY = leftY * REGULAR_FORWARD;
    rightX = rightX * REGULAR_TURN;
  }

  if (flipDrive)
    leftY *= -1;
  // move the chassis with arcade drive
  chassis.arcade(leftY, rightX);
}

// TODO - need to test this
void set_braking(bool brakeCoast = true) {
  if (brakeCoast) {
    leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  } else {
    leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  }
}

void autoDisabled() {
  // do nothing
}

struct Auto {
  std::string name;
  void (*function)(void);
  lv_color_t color;
};

Auto autoFarAuton{"Auto Far", autoFar, LV_COLOR_RED};
Auto autoCloseAuton{"Auto Close", autoClose, LV_COLOR_BLUE};
Auto autoSkillsAuton{"Auto Skills", autoSkills, LV_COLOR_GREEN};
Auto autoDisabledAuton{"Disabled", autoDisabled, LV_COLOR_BLACK};

std::vector<Auto> autos = {
    autoFarAuton, autoCloseAuton, autoSkillsAuton, autoDisabledAuton}; // MUST BE LESS THAN 10 AUTOS AND MORE THAN -1 AUTOS
int currentAuto = 0;
void initialize() {
  pros::delay(500); // Stop the user from doing anything while
                    // legacy ports configure.
  chassis.calibrate();
  pros::lcd::initialize();
}

void writeAuto() {
  FILE *usd_file_write = fopen("/usd/auto.txt", "w");
  std::string cp_str = std::to_string(currentAuto);
  char const *cp_c = cp_str.c_str();
  fputs(cp_c, usd_file_write);
  fclose(usd_file_write);
}

void pgUp() {
  currentAuto = (currentAuto - 1) % autos.size();
  pros::lcd::print(0, "%s", autos[currentAuto].name);
  pros::lcd::set_background_color(autos[currentAuto].color);
  writeAuto();
  pros::delay(500);
}
void pgDown() {
  currentAuto = (currentAuto + 1) % autos.size();
  pros::lcd::print(0, "%s", autos[currentAuto].name);
  pros::lcd::set_background_color(autos[currentAuto].color);
  writeAuto();
  pros::delay(500);
}

void competition_initialize() {
  pros::ADIDigitalIn limit_left('c');
  pros::ADIDigitalIn limit_right('d');
  pros::lcd::register_btn0_cb(pgDown);
  pros::lcd::register_btn2_cb(pgUp);
  pros::lcd::print(0, "%s", autos[currentAuto]);

  if (pros::c::usd_is_installed()) {
    FILE *usd_file_read = fopen("/usd/auto.txt", "r");
    if (usd_file_read != nullptr) {
      char buf[5];
      fread(buf, 1, 5, usd_file_read);
      if (isdigit(buf[0])) {
        currentAuto = std::stof(buf);
      }
      fclose(usd_file_read);
    } else {
      writeAuto();
      pros::lcd::print(1, "%s", "Created File");
    }
  } else {
    pros::lcd::print(1, "%s", "No SD Card");
  }

  while (true) {
    if (limit_left.get_value()) {
      pgUp();
    } else if (limit_right.get_value()) {
      pgDown();
    }
    pros::delay(20);
  }
}

void autonomous() { ((void (*)())autos[currentAuto].function)(); }

void opcontrol() {
  int cataHeadStart = 0;

  // TODO - auto assistance at the start of driver skills
  // call auto close and fire cata

  bool flipDrive = false;
  bool wingState = LOW;  // wings wingState
  bool cataFire = false; // toggle for catapult

  int delayWings = 0;
  int delayFlip = 0;

  while (true) {

    // wings
    if (delayWings) {
      delayWings--;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      wingState = !wingState;
      wings.set_value(wingState);
      delayWings = 40;
    }

    // cata
    // cataDown = limit_switch.get_value();
    // cataDown = pot.get_value() > CATA_THRESHOLD;  // we are using the limit
    // switch

    if (cataHeadStart > 0) {
      cata = CATAMAXVOLTAGE;
      cataHeadStart--;
    }

    // cata toggle
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      cataFire = !cataFire;
    }

    if (cataFire) {
      cata = CATAMAXVOLTAGE; // continuous fire
    } else {
      cata.brake(); // coast up
    }

    // intake
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake = 127;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake = -127;
    } else {
      intake.brake();
    }

    // filpDrive
    if (!delayFlip) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        flipDrive = !flipDrive;
        delayFlip = 40;
      }
    } else {
      delayFlip--;
    }

    arcade_drive(flipDrive);
    pros::delay(20);
  }
}