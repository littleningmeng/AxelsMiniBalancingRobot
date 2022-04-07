
// I2C address of the IMU
const int MPU6050_ADDRESS = 0x68;

// the loop time of the main loop. 
const int STD_LOOP_TIME = 5000;  // 5000 microseconds gives about 200Hz
const float TARGET_DELTA_TIME = STD_LOOP_TIME / 1000000.0; // in seconds

// PID constants for the angle PID controller
const float ANGLE_P = 18; //18.0;          
const float ANGLE_I = 1.2; //1.5;           
const float ANGLE_D = 0.0;           
const float ANGLE_I_LIMIT = 200;      

// PID constants for the speed PID controller
const float SPEED_P = 900.0;          
const float SPEED_I = 2.0;          
const float SPEED_D = 0.0;          
const float SPEED_I_LIMIT = 200;      

// The robot will stop balancing if the angle error between the 
// target angle and the current angle gets larger than this value
const float MAX_ACCEPTABLE_ANGLE_ERROR = 35; 

// The robot will stop balancing it at leans more than this angle. 
const float MAX_ACCEPTABLE_ANGLE = 40; 

// if the angle error is less than this value, the robot will start balancing.
const float START_ANGLE_ERROR = 1; 

// it the battery voltage goes below this value, the robot will stop balancing. 
const float MIN_BAT_VOLTAGE = 10.0f;

// motor acceleration
const float SPEED_ACCELERATION = 0.008; 
const float TURN_ACCELERATION = 0.005; 

// overspeed brakeing: lean back and try to brake if the wheel speed gets to high. 
const float OVERSPEED_TRESHOLD = 1.2f;     // brake if speed is above this value. 
const float OVERSPEED_BRAKE_AMOUNT = 0.40f;   // higher value -> more aggresive brakeing

// abstacle avoidance configuration
// the value from the sensors gets higher when an abstacle is closer
const int TURN_ADJUST_TRESHOLD = 150; // higher than this value causes the robot to slightly turn away from an obstacle while going forward. 
const int STOP_TRESHOLD = 500;        // higher than this value causes the robot to stop going forward.
const int TURNAWAY_TRESHOLD = 400;    // higher than this value causes the robot to make a turn with random duration

const int TURNAWAY_BASE_AMOUNT = 80;    // the static amount to turn away when abstacle is to close
const int TURNAWAY_RANDOM_AMOUNT = 300; // the maximum random amount to turn away when abstacle is to close, 
                                          // those two values are added togheter to calculate the amount to turn away

const float TURNAWAY_SPEED = 0.5f;      // the turnaway speed in wheel rotations per second
const float TURN_ADJUST_MULTIPLYER = 0.0008f; // the diffeance of the two sensor values are multiplyed with this value to calculate the 
                                              // turning speed used to slightly turn away from an obstacles


