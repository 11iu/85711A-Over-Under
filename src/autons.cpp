#include "autons.hpp"
#include "field.hpp"
#include "main.h"
#include "constants.hpp" //for pin numbers

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
void autoAttackBlue()
{
    chassis.setPose(blueStartLower.x, blueStartLower.y, blueStartLowerHeading);
    chassis.moveToPose(fieldX / 2, blueStartLower.y, 180, 4000); // Moves to face the goal
    intake = -127;                                               // score preload
    pros::delay(1500);
    chassis.moveToPose(fieldX / 2, blueStartLower.y - tile / 2, 180, 4000); // Shoves the triball in
    intake = 0;
    chassis.moveToPose(fieldX / 2, blueStartLower.y + tile, 0, 4000); // Move backwards
}

// start in farthest full starting tile, facing the center of the field
void autoAttackRed()
{
    chassis.setPose(redStartUpper.x, redStartUpper.y, redStartUpperHeading);
    chassis.moveToPose(fieldX / 2, redStartUpper.y, 0, 4000); // Moves to face the goal
    intake = -127;                                            // score preload
    pros::delay(1500);
    chassis.moveToPose(fieldX / 2, redStartUpper.y + tile / 2, 0, 4000); // Shoves the triball in
    intake = 0;
    chassis.moveToPose(fieldX / 2, redStartUpper.y - tile, 180, 4000); // Move backwards
}

// start in closest full starting tile, facing center of the field
// remove triball that is in the match load area and touch elevation bar
void autoDefenseBlue()
{
    // TODO - remove triball from the match load area with wing
    chassis.setPose(blueStartUpper.x, blueStartUpper.y, blueStartUpperHeading);
    chassis.moveToPose(blueElevationHorizontalMid.x, blueElevationHorizontalMid.y, 180, 4000);
}

// start in closest full starting tile, facing center of the field
// remove triball that is in the match load area and touch elevation bar
void autoDefenseRed()
{
    // TODO - remove triball from the match load area with wing
    chassis.setPose(redStartLower.x, redStartLower.y, redStartLowerHeading);
    chassis.moveToPose(redElevationHorizontalMid.x, redElevationHorizontalMid.y, 0, 4000);
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
    pros::Motor cata(CATA);
    cata = CATAVOLTAGE;
    pros::delay(10000); // wait 10 sec
    cata = 0;

    // TODO - find optimal angle and create pose for that, add driving and scoring with wings
}