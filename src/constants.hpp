// ports
#define LEFT_BACK_PORT 7
#define LEFT_FRONT_UPPER_PORT 6
#define LEFT_FRONT_LOWER_PORT 10
#define RIGHT_BACK_PORT 5
#define RIGHT_FRONT_UPPER_PORT 2
#define RIGHT_FRONT_LOWER_PORT 1

#define IMU_PORT 4
#define INTAKE_PORT 18
#define CATA_PORT 8
#define WINGS 'a'
#define VERT_WINGS 'd'

#define LEFT_BUMP 'b'
#define RiGHT_BUMP 'c'

#define BACK_ULTRASONIC_IN 'f'  // not used
#define BACK_ULTRASONIC_OUT 'e' // not used
#define INTAKE_ULTRASONIC_IN 'h'
#define INTAKE_ULTRASONIC_OUT 'g'
#define DISTANCE_RIGHT 3

#define LED_PORT 'e'
#define LED_LENGTH 64

// chassis
const int JOYSTICK_THRESHOLD_AMT = 15;

// catapult
const int CATA_THRESHOLD = 2250;
const int CATAMAXVOLTAGE = -127;
const int CATAHOLDVOLTAGE = 0;

// driving multipliers
const double FORWARD_AMT = 1.0;
const double TURN_AMT = 0.8;