#include "autons.hpp"
#include "field.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/imu.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <utility>

Autons::Autons(lemlib::Chassis &chassis, pros::ADIDigitalOut &wings,
               pros::ADIDigitalOut &vertWings, pros::Motor &intake,
               pros::Motor &cata, pros::Distance &distRight,
               pros::ADIUltrasonic &distBack, pros::ADIUltrasonic &distIntake)
    : chassis(chassis), wings(wings), vertWings(vertWings), intake(intake),
      cata(cata), distRight(distRight), distBack(distBack),
      distIntake(distIntake) {}

///////////////////////////////////////////////////
// Robot state functions
///////////////////////////////////////////////////

std::pair<float, float> Autons::localizeRobot()
{
  float conversionFactor = 0.0393701; // converts from mm to inches

  // offsets from sensor to center of robot
  float x_offset = 5;
  float y_offset = 3;

  int samples = 20;
  int detected_samples = 0;
  float x = 0.0;
  float y = 0.0;

  for (int i = 0; i < samples; i++)
  {
    float x_new = fieldX - (distRight.get() * conversionFactor) -
                  x_offset; // returns 0 if not found
    float y_new = distBack.get_value() * conversionFactor +
                  y_offset; // returns 0 if not found

    if (x_new != 0 && y_new != 0)
    {
      x += x_new;
      y += y_new;
    }
    else
    {
      break;
    }

    pros::delay(5);
  }

  // failed to detect
  if (x < 1 || y < 1)
  {
    return std::make_pair(5.5, 3.5);
  }

  x += 5.5;
  y += 3.5;

  return std::make_pair(x, y);
}

bool Autons::hasTriball()
{
  int threshold = 200;
  // return distIntake.get_value() < threshold;
  return false; // placeholder not using the distance sensor
}

void Autons::fireCata()
{
  /*
  u_int32_t start = pros::millis();
  u_int32_t timeout = 2000;

  while (hasTriball() && (pros::millis() - start < timeout))
  {
    intake = 127;
    pros::delay(200);
  }
  intake = 0;
  */

  cata = CATAMAXVOLTAGE;
}

// wings and vert wings
bool wingState = false;
bool vertWingState = false;

void Autons::toggleWings()
{
  wingState = !wingState;
  wings.set_value(wingState);
}
void Autons::toggleVertWings()
{
  vertWingState = !vertWingState;
  vertWings.set_value(vertWingState);
}

void Autons::setWings(bool state) { wings.set_value(state); }
void Autons::setVertWings(bool state) { vertWings.set_value(state); }

///////////////////////////////////////////////////
// Autos Functions
///////////////////////////////////////////////////
ASSET(path_txt);

// starts at opposite of close side facing towards goal, pushes triball into
// the goal, and sets up for match load
void Autons::autoCloseOpposite()
{
  chassis.setPose(closeOppStart.x, closeOppStart.y, closeOppStart.angle);
  chassis.moveToPose(blueGoalLeftSide.x + 2, 13, 90, 1500,
                     {.minSpeed = 80}, false); // push into the goal
  intake = 127;

  // reshove in
  chassis.moveToPose(blueGoalLeftSide.x - tile, 13, 90, 1000,
                     {.forwards = false, .minSpeed = 80}, false);
  chassis.moveToPose(blueGoalLeftSide.x + 2, 13, 90, 1000,
                     {.minSpeed = 100}, false); // push again
  chassis.moveToPose(closeOppEnd.x, closeOppEnd.y, closeOppEnd.angle, 1500,
                     {.forwards = false, .maxSpeed = 80}, false);
  intake = 0;
}

// starts at close side facing towards goal, pushes triball into the goal, and
// sets up for match load
void Autons::autoClose()
{
  chassis.setPose(closeStart.x, closeStart.y, closeStart.angle);
  chassis.moveToPose(blueGoalRightSide.x + 2, 13, -90, 1500,
                     {.minSpeed = 80}, false);
  intake = 127;
  // reshove in
  chassis.moveToPose(blueGoalRightSide.x + tile, 13, -90, 1000,
                     {.forwards = false, .minSpeed = 80}, false);
  chassis.moveToPose(blueGoalRightSide.x - 4, 13, -90, 1000,
                     {.minSpeed = 100}, false);
  chassis.moveToPose(closeEnd.x, closeEnd.y, closeEnd.angle, 1500,
                     {.forwards = false, .maxSpeed = 80}, false);
  intake = 0;
}

// same pos as auto close but backwards
void Autons::autoCloseBackwards()
{
  chassis.setPose(closeStart.x, closeStart.y, closeStart.angle + 180);
  chassis.moveToPose(blueGoalRightSide.x - 2, 13, 90, 1500, {.forwards = false, .minSpeed = 80}, false);
  chassis.setPose(blueGoalRightSide.x + botLength / 2.0, 13, 90);
}

// same pos as auto close but backwards
void Autons::autoCloseBackwardsAnnoying()
{
  autoCloseBackwards();
  chassis.moveToPose(fieldX - tile, tile * 1.5, -90, 1500, {.minSpeed = 80}, false); // move towards center of field
  chassis.moveToPose(fieldX + 8, tile * 2, -90, 1500, {.minSpeed = 60}, false);      // park in the neutral zone and mess up the center triball
}

// start in farthest full starting tile, facing the center of the field
// starts at upper
void Autons::autoFar()
{
  chassis.setPose(farStart.x, farStart.y, farStart.angle);
  chassis.moveToPose(fieldX / 2, farStart.y, farStart.angle, 2000,
                     {.minSpeed = 80}, false); // Moves to in front of goal
  chassis.moveToPose(fieldX / 2, farStart.y, 0, 1000, {},
                     false); // turn to face goal
  intake = 127;
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - 6, 0, 1500,
                     {.minSpeed = 100}, false); // Shoves preload in
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 1500,
                     {.forwards = false}, false); // back out
  intake = 0;
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 1000, {},
                     false); // turn towards center

  // grab lower triball and score
  intake = -127;
  chassis.moveToPose(blueCenterLowerTriball.x - 3, blueCenterLowerTriball.y + 4,
                     180, 2000, {}, false); // move into the triball
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 1000,
                     {.forwards = false}, false); // face goal
  intake = 127;
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - 6, 0, 2000,
                     {.minSpeed = 100}, false); // score
  intake = 0;

  // reset for teleop
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 1500,
                     {.forwards = false}, false); // back out

  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 1000, {},
                     false); // turn towards center

  // give a backwards shove
  chassis.moveToPose(fieldX / 2.0, redGoalCenter.y - 2, 180, 1000,
                     {.forwards = false}, false);
  chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 1000, {}, false);
}

// 4 triball auto lmao
// set up like close opposite(~1 inch gap from wing to angled bar)
void Autons::autoFarInsane()
{
  // score preload
  chassis.setPose(farInsaneStart.x, farInsaneStart.y, farInsaneStart.angle);
  chassis.moveToPose(blueGoalCenter.x - tile * 0.4, farInsaneStart.y, farInsaneStart.angle, 2000, {.minSpeed = 100}, false); // Moves to in front of goal
  chassis.moveToPose(blueGoalCenter.x - tile * 0.4, farInsaneStart.y, 180, 1000, {.minSpeed = 80}, false);                   // turn to face goal
  intake = 127;
  chassis.moveToPose(blueGoalCenter.x - tile * 0.4, blueGoalCenter.y + 2, 180, 1500, {.minSpeed = 100}, false);                      // Shoves preload in
  chassis.moveToPose(blueGoalCenter.x - tile * 0.4, blueGoalCenter.y + tile, 180, 1500, {.forwards = false, .minSpeed = 80}, false); // back out
  chassis.moveToPose(blueGoalCenter.x - tile * 0.4, blueGoalCenter.y + tile, 0, 1500, {.minSpeed = 80}, false);                      // turn towards triball
  intake = 0;

  // pick up triball to the left side
  intake = -127;
  chassis.moveToPose(redCenterLeftTriball.x + 4, redCenterLeftTriball.y - 10, 0, 800, {.maxSpeed = 100}, false); // pick up
  intake = 0;

  // moving backwards to get that lower triball with wings
  chassis.moveToPose(redCenterLeftTriball.x + 4, redCenterLeftTriball.y - 10, -45, 800, {.forwards = false}, false);         // turn
  chassis.moveToPose(redCenterLowerTriball.x - tile * 0.5, tile * 2, -45, 1000, {.forwards = false, .minSpeed = 60}, false); // back up into the lower triball
  setWings(HIGH);
  chassis.moveToPose(redCenterLowerTriball.x - tile * 0.5, tile * 2, 45, 1000, {}, false); // turn a lot to swing the triball around

  // score the left and lower triballs
  chassis.moveToPose(blueGoalCenter.x, tile * 2.5, 0, 1200, {.minSpeed = 80}, false);   // move forward
  chassis.moveToPose(blueGoalCenter.x, tile * 2.5, 180, 1000, {.minSpeed = 80}, false); // turn back around
  intake = 127;
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y, 180, 2000, {.minSpeed = 100}, false); // score
  intake = 0;
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y + tile, 180, 1200, {.forwards = false, .minSpeed = 80}, false); // back out
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y + tile, 0, 800, {}, false);                                     // turn towards center

  // get the triball closer to center and score
  intake = -127;
  chassis.moveToPose(redCenterUpperTriball.x + 4, redCenterUpperTriball.y - 12, 0, 1500, {.maxSpeed = 100}, false); // get the upper triball
  // setWings(LOW);
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y + tile, 0, 2000, {.forwards = false, .minSpeed = 80}, false); // back up
  intake = 0;
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y + tile, 180, 1000, {.forwards = false, .minSpeed = 100}, false); // turn towards goal
  intake = 127;
  // setWings(HIGH);
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y - 2, 180, 2000, {.minSpeed = 100}, false); // score

  // reset for teleop
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y + tile, 180, 2000, {.forwards = false, .minSpeed = 100}, false); // reset
  intake = 0;
  chassis.moveToPose(blueGoalCenter.x, blueGoalCenter.y + tile * 0.5, 0, 2000, {.minSpeed = 100}, false); // reset
}

// starts in far side facing the goal, completes awp(score, descore, touch bar)
// start parallel to angled bar with back of robot towards side of goal
void Autons::autoAWP()
{
  autoCloseBackwards();
  chassis.moveToPose(blueGoalRightSide.x + tile, tile / 2.0, -135, 1500, {}, false); // back up and turn to face the other way
  setVertWings(HIGH);
  chassis.moveToPose(fieldX, tile * 1.25, -135, 2000, {.forwards = false}, false); // back up
  setVertWings(LOW);
  chassis.setPose(fieldX - tile / 2.0, tile + botWidth / 2.0, -135);
  // chassis.moveToPose(fieldX - tile / 2.0, tile * 1.5, -90, 1500, {}, false);                                          // forward a bit
  chassis.moveToPose(fieldX - 16, tile * 1.5, 0, 1500, {}, false);                                    // turn
  chassis.moveToPose(fieldX - 16, redElevationHorizontalMid.y - 6, 0, 1500, {.minSpeed = 60}, false); // go to bar
}

// start the same as autoClose
void Autons::autoSkills()
{
  Autons::autoClose();
  chassis.tank(-5, -10); // push back to mitigate cata momentum
  fireCata();
  pros::delay(28000); // change to 28000 for skills
  cata = 0;
  chassis.tank(0, 0);

  /*
//ram to align
chassis.moveToPose(fieldX - tile * 1.2, tile * 1.2, -45, 2000,
                    {.minSpeed = 80}, false);
chassis.moveToPose(fieldX - tile * 0.6, tile * 0.6, -45, 2000,
                    {.forwards = false, .minSpeed = 80}, false);
chassis.tank(-60, -60);
pros::delay(1500);
chassis.tank(0, 0);
chassis.setPose(fieldX - tile * 0.6, tile * 0.6, -45);
*/

  /*
//localizing position
chassis.moveToPose(fieldX - tile, tile, 0, 2000, {},
    false); // make sure robot parallel with walls for calibration

std::pair<float, float> pos = localizeRobot();
if (pos.first != 0.0 && pos.second != 0.0) {
  chassis.setPose(pos.first, pos.second, chassis.getPose().theta);
}

else { // recalibrate by ramming into angled corner bar

  // if sensor fail
  chassis.moveToPose(fieldX - tile * 1.2, tile * 1.2, -45, 2000,
                      {.minSpeed = 80}, false);
  chassis.moveToPose(fieldX - tile * 0.6, tile * 0.6, -45, 2000,
                      {.forwards = false, .minSpeed = 80}, false);
  chassis.setPose(fieldX - tile * 0.9, tile * 0.9, -45);
}
*/

  // go to the other side and push into right side of goal
  chassis.moveToPose(fieldX - 12, tile * 1.4, 45, 1000,
                     {}, false); // setup for move
  chassis.moveToPose(fieldX - 12, tile * 1.4, 0, 1000,
                     {}, false); // setup for move
  pros::delay(200);
  chassis.moveToPose(fieldX - 12, fieldY / 2.0, 0, 1000,
                     {.minSpeed = 80},
                     false); // cross to the middle under the bar
  chassis.moveToPose(fieldX - 12, fieldY - tile, 0, 1500,
                     {.minSpeed = 80}, false); // go over fully
  chassis.moveToPose(fieldX - 12, fieldY - tile, -60, 1000,
                     {.minSpeed = 80}, false); // turn towards right side of goal
  intake = 127;
  chassis.moveToPose(redGoalRightSide.x + 2, redGoalRightSide.y - 2, -90, 2000,
                     {.minSpeed = 100}, false); // right side push in
  chassis.moveToPose(redGoalRightSide.x + tile * 0.5, redGoalRightSide.y - 2, -90,
                     1500, {.forwards = false, .minSpeed = 80}, false); // back up
  chassis.moveToPose(redGoalRightSide.x + tile * 0.5, redGoalRightSide.y - 2, -180,
                     1000, {}, false); // turn towards center
  chassis.moveToPose(redGoalRightSide.x + tile * 0.5, fieldY / 2.0 + botLength / 2.0, -180,
                     1500, {.minSpeed = 80}, false);
  intake = 0;

  // push in from right center
  chassis.moveToPose(redGoalRightSide.x + tile * 0.5, fieldY / 2.0 + botLength / 2.0, -20, 1000, {},
                     false); // turn towards goal
  setWings(HIGH);
  intake = 127;
  chassis.moveToPose(redGoalCenter.x + tile / 2.0, redGoalCenter.y - 2, -20, 2000,
                     {.minSpeed = 120}, false);
  intake = 0;
  setWings(LOW);

  // push in from center center
  chassis.moveToPose(redGoalCenter.x + tile, fieldY / 2.0 + botLength / 2.0, -20, 2000,
                     {.forwards = false}, false); // back up
  setWings(HIGH);
  intake = 127;
  chassis.moveToPose(redGoalCenter.x + tile / 2.0, redGoalCenter.y - 2, 0, 2000,
                     {.minSpeed = 120}, false);
  intake = 0;
  setWings(LOW);

  // push in from left center
  chassis.moveToPose(fieldX - tile * 2, fieldY / 2.0 + botLength / 2.0, -90, 2000,
                     {.forwards = false}, false); // go back to right side
  chassis.moveToPose(redGoalCenter.x - tile, fieldY / 2.0 + botLength / 2.0, -90, 1500, {.minSpeed = 80},
                     false); // go along center pipe to the left side
  chassis.moveToPose(redGoalCenter.x - tile / 2.0, fieldY / 2.0 + botLength / 2.0, 20, 1000, {},
                     false); // turn towards goal
  setWings(HIGH);
  intake = 127;
  chassis.moveToPose(redGoalCenter.x - tile / 2.0, redGoalCenter.y - 2, 0, 2000,
                     {.minSpeed = 120}, false); // shove into goal
  intake = 0;
  setWings(LOW);

  /*
// recalibrating x with sensor and y by ramming
chassis.moveToPose(fieldX / 2.0, fieldY / 2.0, 0, 2000,
                    {.forwards = false, .minSpeed = 100}, false);

std::pair<float, float> posCenter = localizeRobot();
if (posCenter.first != 0.0) {
  // localize the robot again for x, y is localized by ramming
  chassis.setPose(posCenter.first, fieldY / 2.0 + botLength / 2.0,
                  chassis.getPose().theta);
} else {
  // guess that we are at the center of x
  chassis.setPose(fieldX / 2.0, fieldY / 2.0 + botLength / 2.0,
                  chassis.getPose().theta);
}
*/

  // push triballs in into the left side of the goal
  chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - tile / 2.0, -90, 1000, {.forwards = false},
                     false); // line up towards left
  chassis.moveToPose(tile * 0.5, redGoalCenter.y - tile / 2.0, -90, 2000,
                     {.minSpeed = 100}, false); // drive towards left
  chassis.moveToPose(tile * 0.5, redGoalCenter.y - tile / 2.0, 45, 1000,
                     {}, false); // turn towards left of goal
  intake = 127;
  chassis.moveToPose(redGoalLeftSide.x - 2, redGoalLeftSide.y - 2, 90, 2000,
                     {.minSpeed = 120}, false);
  intake = 0;
  // back up and ram again
  chassis.moveToPose(tile, fieldY - tile, 60, 2000, {.forwards = false}, false);
  intake = 127;
  chassis.moveToPose(redGoalLeftSide.x, redGoalLeftSide.y - 2, 90, 2000,
                     {.minSpeed = 120}, false);
  intake = 0;

  // back up and ram again
  chassis.moveToPose(tile * 0.5, fieldY - tile * 1.5, 60, 2000, {.forwards = false}, false);
  intake = 127;
  chassis.moveToPose(redGoalLeftSide.x + 4, redGoalLeftSide.y - 2, 90, 2000,
                     {.minSpeed = 120}, false);
  intake = 0;

  // back up clear of the other triballs
  chassis.moveToPose(redGoalLeftSide.x - tile, redGoalLeftSide.y - 12, 60, 1000,
                     {.forwards = false}, false);
}

void Autons::autoDisabled()
{
  // do nothing
}

void Autons::autoTest()
{
  // test pid
  chassis.setPose(0, 0, 0);

  for (int i = 0; i < 6; i++)
  {
    chassis.moveToPose(0, 2 * tile, 0, 2000);
    chassis.moveToPose(0, 0, 0, 2000, {.forwards = false});
    chassis.moveToPose(0, 0, 180, 2000);
    chassis.moveToPose(0, 0, 0, 2000);
  }
  /*
chassis.follow(path_txt, 15, 60000, true, false);
chassis.waitUntil(35);
intake = 127;
chassis.waitUntil(40);
intake = 0;
chassis.waitUntil(53);
fireCata();
pros::delay(20); // change to 30000 for skills
cata = 0;
chassis.waitUntil(266);
setWings(HIGH);
chassis.waitUntil(304);
setWings(LOW);
chassis.waitUntil(342);
setWings(HIGH);
chassis.waitUntil(393);
setWings(LOW);
chassis.waitUntil(437);
setWings(HIGH);
chassis.waitUntil(462);
setWings(LOW);
*/
}
