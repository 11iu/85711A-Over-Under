#include "main.h"
#include "lemlib/api.hpp"
#include "field.hpp"
#include "constants.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

//  chassis motors
pros::Motor lF(LEFT_FRONT_LOWER_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor lM(LEFT_FRONT_UPPER_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor lB(LEFT_BACK_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rF(RIGHT_FRONT_LOWER_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rM(RIGHT_FRONT_UPPER_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rB(RIGHT_BACK_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::MotorGroup leftMotors({lF, lM, lB});	// Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup rightMotors({rF, rM, rB}); // Creates a motor group with forwards port 4 and reversed ports 4 & 6

// Inertial Sensor
pros::Imu imu(IMU_PORT);

// drivetrain settings
lemlib::Drivetrain drivetrain(
	&leftMotors,				// left motor group
	&rightMotors,				// right motor group
	10,							// 10 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
	360,						// drivetrain rpm is 360
	2							// chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings
	linearController(10,  // proportional gain (kP)
					 0,	  // integral gain (kI)
					 3,	  // derivative gain (kD)
					 3,	  // anti windup
					 1,	  // small error range, in inches
					 100, // small error range timeout, in milliseconds
					 3,	  // large error range, in inches
					 500, // large error range timeout, in milliseconds
					 20	  // maximum acceleration (slew)
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
					  0	   // maximum acceleration (slew)
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
	&imu	 // inertial sensor
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);
pros::ADIDigitalOut wings(WINGS, LOW);
pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEARSET_18, false,
				   pros ::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata(CATA_PORT, pros::E_MOTOR_GEARSET_36, true);

// autonomous functions

void autoTest()
{
	// example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set
	// to 4000 ms
	chassis.moveToPose(20, 15, 90, 4000);
	// example movement: Move to x: 0 and y: 0 and face heading 270, going
	// backwards. Timeout set to 4000ms
	chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
	// cancel the movement after it has travelled 10 inches
	chassis.waitUntil(10);
	chassis.cancelMotion();
	// example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
	// dont turn faster than 60 (out of a maximum of 127)
	chassis.turnTo(45, -45, 1000, true, 60);
	// example movement: Follow the path in path.txt. Lookahead at 15, Timeout set
	// to 4000 following the path with the back of the robot (forwards = false)
	// see line 116 to see how to define a path
	// chassis.follow(example_txt, 15, 4000, false);
	// wait until the chassis has travelled 10 inches. Otherwise the code directly
	// after the movement will run immediately Unless its another movement, in
	// which case it will wait
	chassis.waitUntil(10);
	// wait until the movement is done
	chassis.waitUntilDone();
}

// start in farthest full starting tile, facing the center of the field
void autoAttackBlue()
{
	// chassis.setPose(blueStartLower.x, blueStartLower.y, blueStartLowerHeading);
	chassis.moveToPose(fieldX / 2, blueStartLower.y, 180,
					   4000); // Moves to face the goal
	intake = -127;			  // score preload
	pros::delay(1500);
	chassis.moveToPose(fieldX / 2, blueStartLower.y - tile / 2, 180,
					   4000); // Shoves the triball in
	intake = 0;
	chassis.moveToPose(fieldX / 2, blueStartLower.y + tile, 0,
					   4000); // Move backwards
}

// start in farthest full starting tile, facing the center of the field
void autoAttackRed()
{
	chassis.setPose(redStartUpper.x, redStartUpper.y, redStartUpperHeading);
	chassis.moveToPose(fieldX / 2, redStartUpper.y, 0,
					   4000); // Moves to face the goal
	intake = -127;			  // score preload
	pros::delay(1500);
	chassis.moveToPose(fieldX / 2, redStartUpper.y + tile / 2, 0,
					   4000); // Shoves the triball in
	intake = 0;
	chassis.moveToPose(fieldX / 2, redStartUpper.y - tile, 180,
					   4000); // Move backwards
}

// start in closest full starting tile, facing center of the field
// remove triball that is in the match load area and touch elevation bar
void autoDefenseBlue()
{
	// TODO - remove triball from the match load area with wing
	chassis.setPose(blueStartUpper.x, blueStartUpper.y, blueStartUpperHeading);
	chassis.moveToPose(blueElevationHorizontalMid.x, blueElevationHorizontalMid.y,
					   180, 4000);
}

// start in closest full starting tile, facing center of the field
// remove triball that is in the match load area and touch elevation bar
void autoDefenseRed()
{
	// TODO - remove triball from the match load area with wing
	chassis.setPose(redStartLower.x, redStartLower.y, redStartLowerHeading);
	chassis.moveToPose(redElevationHorizontalMid.x, redElevationHorizontalMid.y,
					   0, 4000);
}

void awp()
{
	//   // do both autoAttack and autoDefense
	//   // start for autoAttack on defense side

	//   // to center of field
	//   chassis.set_drive_pid(48, DRIVE_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_turn_pid(90, TURN_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_drive_pid(24, DRIVE_SPEED);
	//   chassis.wait_until(12);
	//   // shoot async
	//   pros::Task cataUpTask(cataUp);
	//   chassis.wait_drive();
	//   chassis.set_drive_pid(-24, DRIVE_SPEED);
	//   chassis.wait_drive();
	//   cataUpTask.join();                  //  make sure it has shot
	//   pros::Task cataDownTask(cataDown);  // async cata down
	//   // go back to the match load area
	//   chassis.set_turn_pid(-90, TURN_SPEED);
	//   chassis.set_drive_pid(-24, DRIVE_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_turn_pid(-45, TURN_SPEED);
	//   chassis.wait_drive();
	//   pros::Motor intake(INTAKE);
	//   intake = -127;
	//   chassis.set_drive_pid(-24, DRIVE_SPEED);
	//   chassis.wait_drive();
	//   pros::delay(1000);
	//   intake = 0;                             // intake triball
	//   chassis.set_drive_pid(6, DRIVE_SPEED);  // get to the rod
	//   chassis.wait_drive();
	//   chassis.set_turn_pid(45, TURN_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_drive_pid(12, DRIVE_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_turn_pid(90, TURN_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_drive_pid(12, DRIVE_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_turn_pid(-90, TURN_SPEED);
	//   chassis.wait_drive();
	//   chassis.set_drive_pid(72, DRIVE_SPEED);
	//   chassis.wait_until(60);
	//   chassis.set_max_speed(DRIVE_SPEED / 4);  // slow down so we are there
	//   chassis.wait_drive();
}

// shoots all triballs and scores with wings
void autoSkills()
{
	// shoot for 10 sec
	cata = CATAMAXVOLTAGE;
	pros::delay(10000); // wait 10 sec
	cata = 0;

	// TODO - find optimal angle and create pose for that, add driving and scoring
	// with wings
}

// do not use curvature drive it is buggy af
void arcade_drive(bool flipDrive = false)
{
	// if () // TODO: add deadzone
	// get joystick positions
	int leftY = lemlib::defaultDriveCurve(
		master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 2);
	int rightX = lemlib::defaultDriveCurve(
		master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 2);
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

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "initializing");

	pros::lcd::register_btn1_cb(on_center_button);

	// autonomous(); // for testing auto
}

void autonomous()
{

	// autoAttackBlue();
}

void opcontrol()
{

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

		arcade_drive(flipDrive);
		pros::delay(20);
	}
}
