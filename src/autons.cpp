#include "autons.hpp"
#include "field.hpp"
#include "main.h"

void autoTest()
{
    // example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // example movement: Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has travelled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnTo(45, -45, 1000, true, 60);
    // example movement: Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has travelled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Travelled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

// start in farthest full starting tile, facing the center of the field
void autoAttack()
{
    // pros::Task cataDownTask(cataDown); //bring cata down
    //  chassis.set_drive_pid(55, DRIVE_SPEED); //forward to center of goal
    //  chassis.wait_drive();
    //  chassis.set_turn_pid(-90, TURN_SPEED);  // Turns right 90 degrees to face goal
    //  chassis.wait_drive();
    //  pros::Motor intake(INTAKE);
    //  intake = -127; //score preload
    //  pros::delay(1500);
    //  intake = 0;
    //  chassis.set_drive_pid(11, DRIVE_SPEED / 2); //shove triball in
    //  chassis.wait_drive();
}

// remove triball that is in the match load area
// touch elevation bar
// start in closest tile, touching the match load area
// start with no triball
void autoDefense()
{
    // // make sure that the cata is down so we can load the triball
    // // pros::ADIDigitalIn limit_switch(LIMIT);
    // pros::ADIAnalogIn pot(POT);
    // bool cataDown = pot.get_value() > CATA_THRESHOLD;
    // pros::Motor cata(CATA);
    // pros::Motor intake(INTAKE);
    // while (!cataDown) {
    //   cata = CATAVOLTAGE;
    //   pros::delay(util::DELAY_TIME);
    //   cataDown = pot.get_value() > CATA_THRESHOLD;
    // }
    // cata.brake();

    // // get new triball
    // intake = 127;
    // pros::delay(3000);
    // intake = 0;
    // cata = CATAVOLTAGE;
    // // chassis to the rod
    // chassis.set_drive_pid(-18, DRIVE_SPEED);  // get away from match load
    // chassis.wait_drive();
    // chassis.set_turn_pid(45, TURN_SPEED);  // start to the rod
    // chassis.wait_drive();
    // cata = 0;
    // chassis.set_drive_pid(-7, DRIVE_SPEED);
    // chassis.wait_drive();
    // chassis.set_turn_pid(135, TURN_SPEED);
    // chassis.wait_drive();
    // chassis.set_drive_pid(-16, DRIVE_SPEED);
    // chassis.wait_drive();
    // chassis.set_turn_pid(45, TURN_SPEED);
    // chassis.wait_drive();
    // chassis.set_pid_constants(&chassis.headingPID, 11, 0, 0, 0);

    // chassis.set_drive_pid(-32, DRIVE_SPEED);
    // chassis.wait_until(-24);
    // chassis.set_max_speed(DRIVE_SPEED / 4);  // slow down so we are there
    // chassis.wait_drive();
    // chassis.set_pid_constants(&chassis.headingPID, 0, 0, 0, 0);
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

// setup like autoDefense, with triballs galore
void autoSkills()
{
    // // shoot all triballs
    // // go over and score them all with wings

    // // shoot for 10 sec
    // pros::Motor cata(CATA);
    // cata = CATAVOLTAGE;
    // pros::delay(10000);  // wait 10 sec
    // cata = 0;
    // pros::Task cataDownTask(cataDown);  // down so we can go under
    // // go under
    // chassis.set_drive_pid(-6, DRIVE_SPEED);
    // chassis.set_turn_pid(45, TURN_SPEED);  // get away from the match load area and turn to correct heading
}