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

#define LEFT_BUMP 'b'
#define RiGHT_BUMP 'c'

#define BACK_ULTRASONIC_IN 'f'
#define BACK_ULTRASONIC_OUT 'e'
#define INTAKE_ULTRASONIC_IN 'h'  // not used
#define INTAKE_ULTRASONIC_OUT 'g' // not used
#define DISTANCE_RIGHT 3

#define LED_PORT 'g'
#define LED_LENGTH 64

// chassis
#define JOYSTICK_THRESHOLD_AMT 15

// auto drive
#define DRIVE_SPEED \
    100 // This is 110/127 (around 87% of max speed).  We don't suggest making     \
        // this 127. If this is 127 and the robot tries to heading correct, it's   \
        // only correcting by making one side slower.  When this is 87%, it's      \
        // correcting by making one side faster and one side slower, giving better \
        // heading correction.
#define TURN_SPEED 90
#define SWING_SPEED 90

// catapult
#define CATA_THRESHOLD 2250
#define CATAMAXVOLTAGE -127
#define CATAHOLDVOLTAGE 0