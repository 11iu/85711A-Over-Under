#include "./subsystems/leds.hpp"
#include "autons.hpp"
#include "display/lv_misc/lv_color.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

///////////////////////////////////////////////////
// Chassis
///////////////////////////////////////////////////

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
    linearController(15,  // proportional gain (kP) 20 works
                     0,   // integral gain (kI)
                     4,   // derivative gain (kD) 4 works
                     3,   // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     40   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(2,   // proportional gain (kP) 3 works
                      0,   // integral gain (kI)
                      10,  // derivative gain (kD) 15 works
                      3,   // anti windup
                      2,   // small error range, in degrees
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

pros::Distance distRight(DISTANCE_RIGHT);
pros::ADIUltrasonic distBack(BACK_ULTRASONIC_OUT, BACK_ULTRASONIC_IN);
pros::ADIUltrasonic distIntake(INTAKE_ULTRASONIC_OUT, INTAKE_ULTRASONIC_IN);

Leds leds(LED_PORT, LED_LENGTH);

Autons autons(chassis, wings, vertWings, intake, cata, distRight, distBack,
              distIntake);

struct Auto
{
    std::string name;
    std::function<void()> function;
    int color;
};

Auto autoCloseAuton{"Close reg", std::bind(&Autons::autoClose, autons), 0x000000};
// Auto autoFarAuton{"Far", std::bind(&Autons::autoFar, autons), 0x000000};
Auto autoFarInsaneAuton{"Far insane", std::bind(&Autons::autoFarInsane, autons), 0xFF0000};
// Auto autoCloseBackAuton{"Close (backwards)", std::bind(&Autons::autoCloseBackwards, autons), 0x000000};
Auto autoCloseBackAnnoyingAuton{"Close (backwards) annoying", std::bind(&Autons::autoCloseBackwardsAnnoying, autons), 0x000080};
Auto autoAWPAuton{"AWP close", std::bind(&Autons::autoAWP, autons), 0x0000FF};
Auto autoSkillsAuton{"Skills", std::bind(&Autons::autoSkills, autons), 0xFFFF00};
Auto autoDisabledAuton{"Disabled", std::bind(&Autons::autoDisabled, autons), 0x000000};

// Auto autoTestAuton{"Test", std::bind(&Autons::autoTest, autons), 0xFFFFFF};

std::vector<Auto> autos = {autoFarInsaneAuton};
int currentAuto = 0;

///////////////////////////////////////////////////
// Utility Functions
///////////////////////////////////////////////////

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

void pgUp()
{
    currentAuto++;
    if (currentAuto > autos.size() - 1)
        currentAuto = 0;
    pros::lcd::print(0, "%s", autos[currentAuto].name);
    leds.set_all(autos[currentAuto].color);
}

void pgDown()
{
    currentAuto--;
    if (currentAuto < 0)
        currentAuto = autos.size() - 1;
    pros::lcd::print(0, "%s", autos[currentAuto].name);
    leds.set_all(autos[currentAuto].color);
}

// making bs static
void flashing_seizure_static(void *param)
{
    Leds *leds_instance = static_cast<Leds *>(param);
    leds_instance->flashing_seizure(param);
}

///////////////////////////////////////////////////
// Main Functions
///////////////////////////////////////////////////

void initialize()
{
    pros::delay(500); // Stop the user from doing anything while
                      // legacy ports configure.
    pros::lcd::initialize();
    chassis.calibrate();
}

void competition_initialize()
{
    pros::ADIDigitalIn limit_left(LEFT_BUMP);
    pros::ADIDigitalIn limit_right(RiGHT_BUMP);
    pros::lcd::register_btn0_cb(pgDown);
    pros::lcd::register_btn2_cb(pgUp);
    pros::lcd::print(0, "%s", autos[currentAuto].name);
    leds.set_all(autos[currentAuto].color);

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

void autonomous()
{
    leds.clear_all();

    //pros::Task flash_task(flashing_seizure_static);
    autos[currentAuto]
        .function();
}

void opcontrol()
{
    // toggles
    bool flipDrive = false;
    bool cataFire = false;

    // debounce timer
    int delayVertWing = 0;
    int delayWings = 0;
    int delayCata = 0;
    int delayFlip = 0;

    bool intakeOn = false;

    // auto close at start of driver skills
    if (autos[currentAuto].name == autoSkillsAuton.name)
    {
        autoCloseAuton.function();
        // chassis.tank(-5, -10); // push back to mitigate cata momentum
        cataFire = true;
    }
    set_braking();

    leds.clear_all();

    while (true)
    {

        // drive
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int leftY = logDrive(forward, 2);
        int rightX = logDrive(turn, 2);

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
            autons.toggleWings();
            delayWings = 40;
        }

        // wing
        if (delayVertWing)
        {
            delayVertWing--;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        {
            autons.toggleVertWings();
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

        // cata firing
        if (cataFire)
        {
            autons.fireCata();
        }
        else
        {
            cata.brake(); // coast up
        }

        intakeOn = distIntake.get_value() < 10;

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
            if (intakeOn)
            {
                intake = -127;
            }
            else
            {
                intake.brake();
            }
        }

        if (intakeOn)
        {
            leds.set_all(LV_COLOR_GREEN.full);
        }
        else
        {
            leds.clear_all();
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
