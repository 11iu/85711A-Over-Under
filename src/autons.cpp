#include "autons.hpp"
#include "field.hpp"
#include "pros/rtos.hpp"

Autons::Autons(lemlib::Chassis &chassis, pros::ADIDigitalOut &wings,
               pros::ADIDigitalOut &vertWings, pros::Motor &intake,
               pros::Motor &cata, pros::ADIUltrasonic &rightSonic,
               pros::Distance &distanceBack, pros::ADIUltrasonic &intakeSonic)
    : chassis(chassis), wings(wings), vertWings(vertWings), intake(intake),
      cata(cata), rightSonic(rightSonic), distanceBack(distanceBack),
      intakeSonic(intakeSonic) {}

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
        float x_new = fieldX - (rightSonic.get_value() * conversionFactor) -
                      x_offset; // returns 0 if not found
        float y_new = distanceBack.get() * conversionFactor +
                      y_offset; // returns 0 if not found

        if (x_new != 0 && y_new != 0)
        {
            x += x_new;
            y += y_new;
            detected_samples++;
        }

        pros::delay(5);
    }

    // failed to detect
    if (x < 1 || y < 1)
    {
        return std::make_pair(0, 0);
    }

    // average samples
    x /= detected_samples;
    y /= detected_samples;

    return std::make_pair(x, y);
}

bool Autons::hasTriball()
{
    int threshold = 200;
    return intakeSonic.get_value() < threshold;
}

void Autons::fireCata()
{
    u_int32_t start = pros::millis();
    u_int32_t timeout = 2000;

    while (hasTriball() && (pros::millis() - start < timeout))
    {
        intake = 127;
        pros::delay(200);
    }
    intake = 0;

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

// starts at opposite of close side facing towards goal, pushes triball into
// the goal, and sets up for match load
void Autons::autoCloseOpposite()
{
    chassis.setPose(closeOppStart.x, closeOppStart.y, closeOppStart.angle);
    chassis.moveToPose(blueGoalLeftSide.x + 5, blueGoalLeftSide.y, 90, 2000,
                       {.minSpeed = 100}, false); // push into the goal
    intake = 127;
    pros::delay(500);
    chassis.moveToPose(closeOppEnd.x, closeOppEnd.y, closeOppEnd.angle, 2000,
                       {.forwards = false, .maxSpeed = 80}, false);
    intake = 0;
}

// starts at close side facing towards goal, pushes triball into the goal, and
// sets up for match load
void Autons::autoClose()
{
    chassis.setPose(closeStart.x, closeStart.y, closeStart.angle);
    chassis.moveToPose(blueGoalRightSide.x - 5, blueGoalRightSide.y, -90, 2000,
                       {.minSpeed = 100}, false);
    intake = 127;
    pros::delay(500);
    chassis.moveToPose(closeEnd.x, closeEnd.y, closeOppEnd.angle, 2000,
                       {.forwards = false, .maxSpeed = 80}, false);
    intake = 0;
}

// start in farthest full starting tile, facing the center of the field
// starts at upper
void Autons::autoFar()
{
    chassis.setPose(farStart.x, farStart.y, farStart.angle);
    chassis.moveToPose(fieldX / 2, farStart.y, farStart.angle, 4000,
                       {.minSpeed = 80}, false); // Moves to in front of goal
    chassis.moveToPose(fieldX / 2, farStart.y, 0, 4000, {},
                       false); // turn to face goal
    intake = 127;
    chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - 6, 0, 4000,
                       {.minSpeed = 80}, false); // Shoves preload in
    intake = 0;
    chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 4000,
                       {.forwards = false}, false); // back out
    chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 4000, {},
                       false); // turn towards center

    // grab upper triball and score
    intake = -127;
    chassis.moveToPose(blueCenterLowerTriball.x - 3, blueCenterLowerTriball.y + 4,
                       180, 2000, {}, false); // move into the triball
    chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 2000,
                       {.forwards = false}, false); // face goal
    intake = 127;
    chassis.moveToPose(redGoalCenter.x, redGoalCenter.y - 6, 0, 4000,
                       {.minSpeed = 100}, false); // score
    intake = 0;

    // reset for teleop
    chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 0, 4000,
                       {.forwards = false}, false); // back out

    chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 3000, {},
                       false); // turn towards center

    // give a backwards shove
    chassis.moveToPose(fieldX / 2.0, redGoalCenter.y - 2, 180, 750,
                       {.forwards = false}, false);
    chassis.moveToPose(fieldX / 2, redGoalCenter.y - tile, 180, 500, {}, false);
}

// starts in far side facing the goal, completes awp
// Created for red side far
void Autons::autoFarAWP() {}

// start the same as autoClose
void Autons::autoSkills()
{
    chassis.setPose(closeStart.x, closeStart.y, closeStart.angle);
    chassis.moveToPose(blueGoalRightSide.x - 5, blueGoalRightSide.y, -90, 2000,
                       {.minSpeed = 100}, false);
    intake = 127;
    pros::delay(500);
    chassis.moveToPose(closeEnd.x, closeEnd.y, closeEnd.angle, 2000,
                       {.forwards = false, .maxSpeed = 80}, false); // TODO: check
    intake = 0;

    //   chassis.tank(0, -20); // push back to mitigate cata momentum
    fireCata();
    u_int32_t start = pros::millis();
    u_int32_t timeout = 30000;
    while (pros::millis() - start < timeout)
    {
        chassis.moveToPose(closeEnd.x, closeEnd.y, closeEnd.angle, 500,
                           {
                               .forwards = false,
                               .maxSpeed = 50,
                               .minSpeed = 25,
                           },
                           false);
    }
    cata = 0;
    chassis.tank(0, 0);

    // localizing position
    chassis.moveToPose(
        fieldX - tile * 0.8, tile * 0.8, 0, 2000, {},
        false); // make sure robot parallel with walls for calibration

    std::pair<float, float> pos = localizeRobot();
    if (pos.first != 0.0 && pos.second != 0.0)
    {
        chassis.setPose(pos.first, pos.second, chassis.getPose().theta);
    }

    else
    {
        // recalibrate our position by ramming backwards into the angled corner bar if sensor fail
        chassis.moveToPose(fieldX - tile * 1.2, tile * 1.2, -45, 2000,
                           {.minSpeed = 80}, false);
        chassis.moveToPose(fieldX - tile * 0.6, tile * 0.6, -45, 2000,
                           {.forwards = false, .minSpeed = 80}, false);
        chassis.setPose(fieldX - tile * 0.9, tile * 0.9, -45);
    }

    // go to the other side and push into right side of goal
    chassis.moveToPose(fieldX - tile / 2.0, tile, 0, 2000, {.minSpeed = 80},
                       false);
    chassis.moveToPose(fieldX - tile / 2.0, fieldY / 2.0, 0, 2000,
                       {.minSpeed = 80}, false);
    chassis.moveToPose(fieldX - tile / 2.0, fieldY - tile * 1.5, 0, 2000,
                       {.minSpeed = 80}, false);
    chassis.moveToPose(fieldX - tile / 2.0, fieldY - tile * 1.5, -60, 2000,
                       {.minSpeed = 80}, false);
    chassis.moveToPose(redGoalRightSide.x - 6, redGoalRightSide.y, -60, 2000,
                       {.minSpeed = 120}, false);

    // ramming into the center from the right, straight on, then left
    chassis.moveToPose(redGoalRightSide.x + tile, redGoalRightSide.y, -160, 3000,
                       {.forwards = false, .minSpeed = 50}, false);
    chassis.moveToPose(fieldX / 2.0 + tile * 1.2, fieldY / 2.0 + tile, -160, 3000,
                       {.minSpeed = 80}, false);
    chassis.moveToPose(fieldX - tile * 2, fieldY / 2.0 + 12, -180, 2000,
                       {.minSpeed = 100}, false); // line up to right of goal
    chassis.moveToPose(fieldX - tile * 2, fieldY / 2.0 + 12, -20, 2000, {},
                       false); // turn towards goal
    setWings(HIGH);
    chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, -20, 2000,
                       {.minSpeed = 120}, false);
    pros::delay(200);

    setWings(LOW);
    chassis.moveToPose(fieldX / 2, fieldY / 2.0 + 12, 0, 2000,
                       {.forwards = false},
                       false); // line up in front of the goal
    setWings(HIGH);
    chassis.moveToPose(redGoalCenter.x - tile / 2.0, redGoalCenter.y, 0, 2000,
                       {.minSpeed = 120}, false);
    pros::delay(200);

    setWings(LOW);
    chassis.moveToPose(tile * 2, fieldY / 2.0 + 12, 20, 2000, {.forwards = false},
                       false); // line up to the left of goal
    setWings(HIGH);
    pros::delay(200);
    chassis.moveToPose(redGoalCenter.x, redGoalCenter.y, 20, 2000,
                       {.minSpeed = 120}, false);
    setWings(LOW);

    // recalibrating x with sensor and y by ramming
    chassis.moveToPose(fieldX / 2.0, fieldY / 2.0, 0, 2000,
                       {.forwards = false, .minSpeed = 100}, false);

    std::pair<float, float> posCenter = localizeRobot();
    if (posCenter.first != 0.0)
    {
        // localize the robot again for x, y is localized by ramming
        chassis.setPose(posCenter.first, fieldY / 2.0 + botLength / 2.0, chassis.getPose().theta);
    }
    else
    {
        // guess that we are at the center of x
        chassis.setPose(fieldX / 2.0, fieldY / 2.0 + botLength / 2.0, chassis.getPose().theta);
    }

    // push triballs in into the left side of the goal
    chassis.moveToPose(tile, fieldY - tile, -45, 3000, {.minSpeed = 80}, false);
    chassis.moveToPose(tile, fieldY - tile, 60, 2000, {}, false);
    chassis.moveToPose(redGoalLeftSide.x, redGoalLeftSide.y, 90, 3000, {.minSpeed = 100}, false);
    // back up and ram again
    chassis.moveToPose(tile, fieldY - tile, 60, 2000, {.forwards = false}, false);
    chassis.moveToPose(redGoalLeftSide.x, redGoalLeftSide.y, 90, 3000, {.minSpeed = 100}, false);
}

void Autons::autoDisabled()
{
    // do nothing
}
