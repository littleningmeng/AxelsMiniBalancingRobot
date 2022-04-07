/*
A small balancing robot using "Axels Stepper Motor Robot Controller"-board. 
The robot uses two stepper motors with A4899 drivers and an MPU-6050 
gyro/accelerometer to measure the angle. This code runs on an Arduino Pro Micro, 
which uses it's two 16 bit hardware timers to generate the step-pulses to drive the motors. 

The robot has two switches connected to digital pins 14 and 15. 
On of the switches controls is the robot should stand still or 
move forward at a constant speed, and the other switch determines 
if the robot should make random turns. 

More info about the robot on my website: http://axelsdiy.brinkeby.se/

Version 1.0

Axel Brinkeby 
2017-09-21
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
int randomTurnCounter = 0;      // used to make random turns
float voltage = 0;              // battery voltage

float motorSpeed = 0;           // the actual forward/reverce speed of the motors. 
int8_t directionMotor1 = 0;
int8_t directionMotor2 = 0;


void setup() {
  
  Serial.begin(115200);         // for debug

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

  Wire.begin();                         // for some reason the main loop runns slower with this line of code
  //Wire.setClock(400000); 
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
void delay_1us()
{
  __asm__ __volatile__ (
    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"
    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"
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
  delay_1us();
  PORTD &= ~(1<<PORTD4); //clear bit    // same as digitalWrite(STEPPER_1_STEP_PIN, LOW);
}

// TIMER 3: controlling motor 2
ISR(TIMER3_COMPA_vect)
{
  TCNT3 = 0;  
  if (directionMotor2 == 0)
    return;
    
  PORTD |= (1<<PORTD7); //set bit       // same as digitalWrite(STEPPER_2_STEP_PIN, HIGH); 
  delay_1us();
  PORTD &= ~(1<<PORTD7); //clear bit    // same as digitalWrite(STEPPER_2_STEP_PIN, LOW);
}


void loop() {
  // hearbeat LED flashing, if you see this LED flashing, the robot is running it's main loop. 
  if (targetSpeed == 0)
    digitalWrite(LED_PIN, millis() % 500 > 400);
  else
    digitalWrite(LED_PIN, millis() % 100 > 50);

  checkBatteryVoltage(); 
  //Serial.println(voltage); 
  if (voltage < MIN_BAT_VOLTAGE) {
    balancing = false;    
    buzzer.buzzBlocking(50);
    delay(200); 
  }

  imu.updateIMUdata(); 

  // calculate the lean angle based in accelerometer and gyro data. This is done with a simple complimentary filter. 
  float accAngle = atan2(imu.getAccelY(), imu.getAccelZ()) * 57;     
  currentLeanAngle = angleFilter.calculate(accAngle, imu.getGyroX(), deltaTime);
  //Serial.println(currentLeanAngle);

  behaviour(); 

  // balance the robot. 
  float targetAngle = speedPID.updatePID(targetSpeed, motorSpeed, deltaTime); 

  /*
  float angleMultiplyer = abs(motorSpeed) * 1.0f;
  if (angleMultiplyer > 1)
    angleMultiplyer = 1; 
    
  targetAngle = targetAngle * angleMultiplyer; 
  */
  
  motorSpeed = -anglePID.updatePID(targetAngle, currentLeanAngle, deltaTime);

  // start balancing if the robot is close to equilibrium. 
  if (!balancing && millis() > 3000 && abs(currentLeanAngle) < START_ANGLE_ERROR) {
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

    // motor speed is converted from rotation per second to steps per second
    float leftMotorSpeed = (motorSpeed + turningSpeed) * 3600;
    float rightMotorSpeed = (motorSpeed - turningSpeed) * 3600; 
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
  //Serial.print(lastLoopUsedTime); Serial.print(" / "); Serial.println(lastLoopTime); 
}

void behaviour() {  
  if (digitalRead(DIGITAL_14_PIN)) {
    targetSpeed = 0.5f;
  } else {
    targetSpeed = 0;
  }

  if (digitalRead(DIGITAL_15_PIN)) {
    if (abs(motorSpeed) < 0.2 && random(0, 2000) <= 1) {
      randomTurnCounter = random(120, 200);
      if (random(0, 2))
        turningSpeed = 0.1f * random(1, 5);
      else
        turningSpeed = -0.1f * random(1, 5);
    }
  }

  if (randomTurnCounter > 0) {
    randomTurnCounter--;
    if (randomTurnCounter == 0) {
      turningSpeed = 0; 
    }
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
  
}
