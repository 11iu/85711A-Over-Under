// ports
#define LEFT_BACK_PORT 7
#define LEFT_FRONT_UPPER_PORT 6
#define LEFT_FRONT_LOWER_PORT 10
#define RIGHT_BACK_PORT 5
#define RIGHT_FRONT_UPPER_PORT 2
#define RIGHT_FRONT_LOWER_PORT 1

#define IMU_PORT 4
#define INTAKE_PORT 9
#define CATA_PORT 8
#define WINGS 'a'
#define VERT_WINGS 'd'

// TODO - change these
#define REAR_ULTRASONIC_PING 0
#define REAR_ULTRASONIC_ECHO 0
#define INTAKE_ULTRASONIC_PING 'e'
#define INTAKE_ULTRASONIC_ECHO 'f'
#define DISTANCE_BACK 0

// chassis
#define JOYSTICK_THRESHOLD_AMT 15

// auto drive
#define DRIVE_SPEED                                                            \
  100 // This is 110/127 (around 87% of max speed).  We don't suggest making
      // this 127. If this is 127 and the robot tries to heading correct, it's
      // only correcting by making one side slower.  When this is 87%, it's
      // correcting by making one side faster and one side slower, giving better
      // heading correction.
#define TURN_SPEED 90
#define SWING_SPEED 90

// catapult
#define CATA_THRESHOLD 2250
#define CATAMAXVOLTAGE -127
#define CATAHOLDVOLTAGE 0

// max speeds
#define REGULAR_FORWARD 1.0
#define REGULAR_TURN 0.8
#define TURBO_FORWARD 1.0
#define TURBO_TURN 1.0