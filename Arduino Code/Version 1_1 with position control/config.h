
// I2C address of the IMU
const int MPU6050_ADDRESS = 0x68;

// the loop time of the main loop. 
const int STD_LOOP_TIME = 3500;  // microseconds
const float TARGET_DELTA_TIME = STD_LOOP_TIME / 1000000.0; // in seconds

// PID constants for the angle PID controller
const float ANGLE_P = 18; //60.0;          
const float ANGLE_I = 1.5; //7.0;           
const float ANGLE_D = 0; //0.0;           
const float ANGLE_I_LIMIT = 200;      

// PID constants for the speed PID controller
const float SPEED_P = 900; //600    450.0;          
const float SPEED_I = 0; //2     4.0;          
const float SPEED_D = 0; //0.0;          
const float SPEED_I_LIMIT = 200;      

// PID constants for the position PID controller
const float POSITION_P = 2;          
const float POSITION_I = 0.12;          
const float POSITION_D = 4;          
const float POSITION_I_LIMIT = 10;  

// The robot will stop balancing if the angle error between the 
// target angle and the current angle gets larger than this value
const float MAX_ACCEPTABLE_ANGLE_ERROR = 20; 

// The robot will stop balancing it at leans more than this angle. 
const float MAX_ACCEPTABLE_ANGLE = 40; 

// if the angle error is less than this value, the robot will start balancing.
const float START_ANGLE_ERROR = 1; 

// it the battery voltage goes below this value, the robot will stop balancing. 
const float MIN_BAT_VOLTAGE = 10.0f;


