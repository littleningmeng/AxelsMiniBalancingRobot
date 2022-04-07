/*
A small balancing robot using "Axels Stepper Motor Robot Controller"-board. 
The robot uses two stepper motors with A4899 (later replaced by TMC2100) drivers and an MPU-6050 
gyro/accelerometer to measure the angle. This code runs on an Arduino Pro Micro, 
which uses it's two 16 bit hardware timers to generate the step-pulses to drive the motors. 

Arduino Pro Micro: https://www.sparkfun.com/products/12640

The robot has two switches connected to digital pins 14 and 15, 
controlling the behaviur of the robot. 

More info about the robot on my website: http://axelsdiy.brinkeby.se/

Version 1.0 - the first published version.  

Version 1.1 - decreased the loop time slightly and tuned all the PID parameters better. 
I am now running TMC2100 stepper motor drivers, which runs smoother than A4988. 
If you are using A4988 drivers, the robot may become unstable with my those PID parameters. 

Version 1.2 - obstacle avoidance using two analog sharp IR sensors. The robot goes slowly forward 
and turns away from obstacles. If abstacles gets to close, the robot will stop and turn 
away in a random direction and speed. This version also has a slightly longer loop time, and also 
retuned PID values in the config.h file. 

Axel Brinkeby 
2017-12-30
*/

#include "pinConfiguration.h"
#include "config.h"
#include "buzzer.h"
#include "mpu6050.h"
#include "complementaryFilter.h"
#include "pid.h"

#include <Wire.h>

long lastLoopTime = STD_LOOP_TIME;
long lastLoopUsedTime = 0;
unsigned long loopStartTime = 0;
float deltaTime = 0;  // unit: seconds

Mpu6050 imu = Mpu6050(MPU6050_ADDRESS);
ComplementaryFilter angleFilter; 
Pid anglePID(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_I_LIMIT);
Pid speedPID(SPEED_P, SPEED_I, SPEED_D, SPEED_I_LIMIT); 
Buzzer buzzer = Buzzer(BUZZER_PIN); 

boolean balancing = false; 
float currentLeanAngle = 0.0f;
float targetSpeed = 0;          // used to control the robot, drive and turn, unit: wheel revulutions per second
float turningSpeed = 0; 
float actualTargetSpeed = 0; 
float actualTurningSpeed = 0; 
int randomTurnCounter = 0;      // used to make random turns
float voltage = 0;              // battery voltage
bool lowVoltageTriggered = false; 
float motorSpeed = 0;           // the actual forward/reverce speed of the motors. 

volatile int8_t directionMotor1 = 0;
volatile int8_t directionMotor2 = 0;

int turnCounter = 0; 

void setup() {  
  Serial.begin(500000);         // for debug

  analogReference(INTERNAL);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STEPPER_1_STEP_PIN, OUTPUT);
  pinMode(STEPPER_1_DIR_PIN, OUTPUT);
  pinMode(STEPPER_2_STEP_PIN, OUTPUT);
  pinMode(STEPPER_2_DIR_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(DIGITAL_14_PIN, INPUT_PULLUP);
  pinMode(DIGITAL_15_PIN, INPUT_PULLUP);

  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // disable the stepper motors

  //buzzer.disable();
  buzzer.buzzBlocking(10); 

// for some reason the main loop runns slower when wire is inisialiced, 
// the communicaton will still work without the init function, 
// but it seam to be unstable, more read errors occur. 
  Wire.begin();                         
  Wire.setClock(400000); 
  imu.init(); 
  imu.calibradeGyro();
  
  // TIMER1 controls motor 1
  TCCR1A = 0;                             // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11);    // Prescaler=8, => 2Mhz
  OCR1A = 65535;                          // longest period, motor stopped
  TCNT1 = 0;

  // TIMER3 controls motor 2
  TCCR3A = 0;                             // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM12) | (1 << CS11);    // Prescaler=8, => 2Mhz
  OCR3A = 65535;                          // longest period, motor stopped
  TCNT3 = 0;

  TIMSK1 |= (1 << OCIE1A);    // Enable TIMER1 interrupt
  TIMSK3 |= (1 << OCIE3A);    // Enable TIMER3 interrupt

  buzzer.buzzBlocking(100); 
}

// 16 single cycle instructions is 1 microsecond at 16Mhz
void delay_05us()
{
  __asm__ __volatile__ (
    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"
    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"    "nop");
}

// TIMER 1: controlling motor 1
ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;  
  if (directionMotor1 == 0)
    return;
    
  PORTD |= (1<<PORTD4); //set bit       // same as digitalWrite(STEPPER_1_STEP_PIN, HIGH); 
  delay_05us();
  PORTD &= ~(1<<PORTD4); //clear bit    // same as digitalWrite(STEPPER_1_STEP_PIN, LOW);
}

// TIMER 3: controlling motor 2
ISR(TIMER3_COMPA_vect)
{
  TCNT3 = 0;  
  if (directionMotor2 == 0)
    return;
    
  PORTD |= (1<<PORTD7); //set bit       // same as digitalWrite(STEPPER_2_STEP_PIN, HIGH); 
  delay_05us();
  PORTD &= ~(1<<PORTD7); //clear bit    // same as digitalWrite(STEPPER_2_STEP_PIN, LOW);
}


void loop() {
  // hearbeat LED flashing, if you see this LED flashing, the robot is running it's main loop. 
  if (digitalRead(DIGITAL_14_PIN))
    digitalWrite(LED_PIN, millis() % 500 > 450);
  else if (digitalRead(DIGITAL_15_PIN))
    digitalWrite(LED_PIN, millis() % 200 > 100);
  else
    digitalWrite(LED_PIN, millis() % 500 > 50);

  checkBatteryVoltage(); 
  //Serial.println(voltage); 
  if (voltage < MIN_BAT_VOLTAGE && millis() > 3000) {
    balancing = false;  
    lowVoltageTriggered = true;   
    buzzer.buzzBlocking(50);
    delay(200); 
  }

  if (lowVoltageTriggered) {
    digitalWrite(LED_PIN, HIGH);
    buzzer.buzzBlocking(50);
    digitalWrite(LED_PIN, LOW);
    delay(1000); 
  }

  imu.updateIMUdata(); 

  // calculate the lean angle based in accelerometer and gyro data. This is done with a simple complimentary filter. 
  float accAngle = atan2(imu.getAccelY(), imu.getAccelZ()) * 57;     
  currentLeanAngle = angleFilter.calculate(accAngle, imu.getGyroX(), TARGET_DELTA_TIME);  
  //Serial.println(currentLeanAngle);

  behaviour(); 
  
  actualTargetSpeed += constrain(targetSpeed - actualTargetSpeed, -SPEED_ACCELERATION, SPEED_ACCELERATION); 
  actualTurningSpeed += constrain(turningSpeed -actualTurningSpeed, -TURN_ACCELERATION, TURN_ACCELERATION);  

  // balance the robot. 
  float targetAngle = speedPID.updatePID(actualTargetSpeed, motorSpeed, TARGET_DELTA_TIME); 
  motorSpeed = -anglePID.updatePID(targetAngle, currentLeanAngle, TARGET_DELTA_TIME);

  // start balancing if the robot is close to equilibrium. 
  if (!lowVoltageTriggered && !balancing && millis() > 3000 && abs(currentLeanAngle) < START_ANGLE_ERROR) {
    balancing = true;  
    turningSpeed = motorSpeed = 0; 
    anglePID.resetPID(); 
    speedPID.resetPID(); 
    buzzer.buzzBlocking(20);
  }
  
  if (balancing) {
    // stop balancing if angle error is to large. 
    if ((targetAngle - MAX_ACCEPTABLE_ANGLE_ERROR) > currentLeanAngle || 
          currentLeanAngle > (targetAngle + MAX_ACCEPTABLE_ANGLE_ERROR)) {
      balancing = false;             
      buzzer.buzzBlocking(50); 
    }

    // stop balancing if the robot is leaning to much in any direction.
    if (-MAX_ACCEPTABLE_ANGLE > currentLeanAngle || currentLeanAngle > MAX_ACCEPTABLE_ANGLE) {
      balancing = false;    
      buzzer.buzzBlocking(50); 
    }
    
    if (motorSpeed > OVERSPEED_TRESHOLD) {
       actualTargetSpeed = -(OVERSPEED_BRAKE_AMOUNT * (motorSpeed - OVERSPEED_TRESHOLD)); 
    } else if (motorSpeed < -OVERSPEED_TRESHOLD) {
       actualTargetSpeed = (OVERSPEED_BRAKE_AMOUNT * (OVERSPEED_TRESHOLD - motorSpeed)); 
    } 

    // motor speed is converted from rotation per second to steps per second
    int16_t leftMotorSpeed = (motorSpeed + actualTurningSpeed) * 3600;
    int16_t rightMotorSpeed = (motorSpeed - actualTurningSpeed) * 3600; 
    setMotorSpeed(leftMotorSpeed, 2);
    setMotorSpeed(rightMotorSpeed, 1);
    digitalWrite(STEPPER_ENABLE_PIN, LOW); // enable the stepper motors
  } 
  else
  {
    setMotorSpeed(0, 2);  // set the speeds of each motor to zero
    setMotorSpeed(0, 1);
    digitalWrite(STEPPER_ENABLE_PIN, HIGH); // disable the stepper motors
  }

  // ===================================== loop timing control =====================================
  lastLoopUsedTime = micros() - loopStartTime;    
  if (lastLoopUsedTime < STD_LOOP_TIME)     
    delayMicroseconds(STD_LOOP_TIME - lastLoopUsedTime);    
  lastLoopTime = micros() - loopStartTime;  
  deltaTime = (float)lastLoopTime / (float)1000000;  
  loopStartTime = micros();    

  // show the used time and total time of the main loop in the serial monitor, note that sending 
  //this data also increses the used time in the main loop. The actual used time is slightly less 
  //than the value showed here. 
  //Serial.print(lastLoopUsedTime); Serial.print(","); Serial.println(lastLoopTime); 
}

void behaviour() {  
  if (digitalRead(DIGITAL_14_PIN)) {          // go forward and turn away from obstacles, if both switches are activated, this is the mode used
    if (turnCounter == 0) {
          
      targetSpeed = 0.5f;   // go slowly forward
  
      int leftsensor = analogRead(ANALOG_3_PIN); 
      int rightsensor = analogRead(ANALOG_2_PIN); 
  
      if (leftsensor > STOP_TRESHOLD || rightsensor > STOP_TRESHOLD) {
        targetSpeed = -0.2f;  // stop and go slightly backwards when obstacle is to close 
      }
      
      // print sensor raw data for debug purpose
      //Serial.print(leftsensor); Serial.print(","); Serial.println(rightsensor); 

      // calculate a turning speed to make the robot turn away from obsticles while going slowly forward. 
      float turn = -(leftsensor - rightsensor) * TURN_ADJUST_MULTIPLYER; 
      
      if (leftsensor > TURNAWAY_TRESHOLD) {   // stop and make a right turn 
        turningSpeed = -TURNAWAY_SPEED;
        turnCounter = TURNAWAY_BASE_AMOUNT + random(1, TURNAWAY_RANDOM_AMOUNT); 
      } else if (rightsensor > TURNAWAY_TRESHOLD) {     // stop and make a left turn
        turningSpeed = TURNAWAY_SPEED;
        turnCounter = TURNAWAY_BASE_AMOUNT + random(1, TURNAWAY_RANDOM_AMOUNT);
      } else if (leftsensor > TURN_ADJUST_TRESHOLD || rightsensor > TURN_ADJUST_TRESHOLD) {   // turn slightly while going forward
        turningSpeed = turn;  
      } else {                // go worward with no turning
        turningSpeed = 0; 
      }
      
    } else {      // turn away from an obstacle that is already detected
      turnCounter--;
      targetSpeed = 0;
    }
    
  } else if (digitalRead(DIGITAL_15_PIN)) {     // stand still in one location and turn around based on sensor data
    
    targetSpeed = 0.0f;
    int leftsensor = analogRead(ANALOG_3_PIN); 
    int rightsensor = analogRead(ANALOG_2_PIN); 
    float turn = -(leftsensor - rightsensor) * TURN_ADJUST_MULTIPLYER; 

    if (leftsensor > TURN_ADJUST_TRESHOLD || rightsensor > TURN_ADJUST_TRESHOLD) {   // turn towards detected obstacle, this way, the robot can follow your hand for example. 
      turningSpeed = -turn * 0.5; 
    } else {                // go worward with no turning
      turningSpeed = 0; 
    }

  }  else {       // stand still in one location, no turning
    targetSpeed = 0;
    turningSpeed = 0; 
  }
}

void checkBatteryVoltage() {
  long value = analogRead(VOLTAGE_SENSE_PIN); 
  voltage = value / 36.80f;
}

void setMotorSpeed(int16_t tspeed, int motorID)
{
  long timer_period;
  int16_t motorspeed = tspeed;

  noInterrupts();
  
  if (motorID == 1) {
    if (motorspeed > 0) {
      timer_period = 2000000 / motorspeed; // 2Mhz timer
      directionMotor1 = 1;
      digitalWrite(STEPPER_1_DIR_PIN, LOW);
    } else if (motorspeed < 0) {
      timer_period = 2000000 / -motorspeed;
      directionMotor1 = -1;
      digitalWrite(STEPPER_1_DIR_PIN, HIGH);
    } else {
      timer_period = 65535;
      directionMotor1 = 0;
    }

    if (timer_period > 65535)   // Check for maximun period without overflow
      timer_period = 65535;

    OCR1A = timer_period;  
    if (TCNT1 > OCR1A)    // Check  if we need to reset the timer...
      TCNT1 = 0; 
    
  } else if (motorID == 2){
    if (motorspeed > 0) {
      timer_period = 2000000 / motorspeed; // 2Mhz timer
      directionMotor2 = 1;
      digitalWrite(STEPPER_2_DIR_PIN, HIGH);
    } else if (motorspeed < 0) {
      timer_period = 2000000 / -motorspeed;
      directionMotor2 = -1;
      digitalWrite(STEPPER_2_DIR_PIN, LOW);
    } else {
      timer_period = 65535;
      directionMotor2 = 0;
    }

    if (timer_period > 65535)   // Check for maximun period without overflow
      timer_period = 65535;
    
    OCR3A = timer_period;  
    if (TCNT3 > OCR3A)    // Check  if we need to reset the timer...
      TCNT3 = 0;    
  }   
  interrupts();   
}
