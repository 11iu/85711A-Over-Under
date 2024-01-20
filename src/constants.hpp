// ports
#define LEFT_BACK 7
#define LEFT_MIDDLE 6
#define LEFT_FRONT 10
#define RIGHT_BACK 5
#define RIGHT_MIDDLE 2
#define RIGHT_FRONT 1

#define IMU 4
#define INTAKE 9
#define CATA 8
#define WINGS 'a'

// chassis
#define JOYSTICK_THRESHOLD 15

// auto drive
#define DRIVE_SPEED                                                            \
  100 // This is 110/127 (around 87% of max speed).  We don't suggest making
      // this 127. If this is 127 and the robot tries to heading correct, it's
      // only correcting by making one side slower.  When this is 87%, it's
      // correcting by making one side faster and one side slower, giving better
      // heading correction.
#define TURN_SPEED 90
#define SWING_SPEED 90

// teleop drive
#define FORWARD_FACTOR 0.75
#define TURN_FACTOR 0.6
#define TURBO_FORWARD_FACTOR 1.0
#define TURBO_TURN_FACTOR 0.8

// catapult
#define CATA_THRESHOLD 2250
#define CATAVOLTAGE -80
#define CATAMAXVOLTAGE -127
#define CATAHOLDVOLTAGE 0
