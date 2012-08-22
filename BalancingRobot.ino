/*
  Selfbalancing Robot  
 
 Created 8 Aug 2012
 Author: Simon Furborg
 
 */

/* Debug flag */
#define DEBUG 1

#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"


/* Defines for registers handling internall pullups */
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/* Accelerometer and Gyro
 * class default I2C address is 0x68 */
MPU6050 accelgyro;

/* xyz values */
int16_t ax, ay, az;
int16_t gx, gy, gz;


/* Default values */
#define DEFAULT_KP 3.18
#define DEFAULT_KI 0.001
#define DEFAULT_KD 50

#define DELTA_TIME 10
#define MAX_ANGLE 150
#define MAX_SPEED 255

/* IO defines */
#define PWM_A     3
#define PWM_B     11
#define BRAKE_A   9
#define BRAKE_B   8
#define DIR_A     12
#define DIR_B     13
#define CURRENT_A 0

#define SMALL_ERROR 4

float kp = DEFAULT_KP;
float ki = DEFAULT_KI;
float kd = DEFAULT_KD;

/* Global variables */
float integral = 0;
float derivate = 0;

float robotAngle  = 0;
float oldAngle    = 0;
float angleOffset = 0;

void setup() {
  /* Initiate debug mode (serial printouts) */
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Hello there, let's start!");
#endif

  /* Initiate I2C bus */
  Wire.begin();

  /* Deactivate internal pullups */  
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
  // deactivate internal pull-ups for twi
  // as per note from atmega8 manual pg167
  cbi(PORTC, 4);
  cbi(PORTC, 5);
#else
  // deactivate internal pull-ups for twi
  // as per note from atmega128 manual pg204
  cbi(PORTD, 0);
  cbi(PORTD, 1);
#endif

  /* initialize device */
  Serial.println("Starting I2C Gyro");
  accelgyro.initialize();

  /* Check connection to i2c device*/
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");

  /* Setup the motor shield */
  /* Setup Motor Channel A */
  pinMode(DIR_A, OUTPUT);    /* Initiates Motor Channel A pin */
  pinMode(BRAKE_A, OUTPUT);  /* Initiates Brake Channel A pin */
  /* Setup Motor Channel B */
  pinMode(DIR_B, OUTPUT);    /* Initiates Motor Channel B pin */
  pinMode(BRAKE_B, OUTPUT);  /* Initiates Brake Channel B pin */
  
  Serial.println("Calibrating angle, find equality for 2 seconds");
  delay(2000);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angleOffset = ax;
}

void loop() {
  int motorSpeed = 0;

  /* Store old angle */
  oldAngle = robotAngle;

  /* Check angle of robot */
  robotAngle = getAngle();

  /* Calculate the speed and direction of motors */
  motorSpeed = calculateMotorSpeed(robotAngle);

  /* Set the speed of motors */
  setMotorSpeed(motorSpeed);

  /* Dealy for DELTA_TIME ms */
  delay(DELTA_TIME);
}



/* 
 * Retrieve angle from sensor 
 */
float getAngle() {
  int angle = 0;

  /* Get "angle" from serial console */
/*#ifdef DEBUG
  angle = Serial.read();
  if (angle == -1)
    angle = 0;
  else if (angle == 65)
    angle = 150;
  else if (angle == 90)    
    angle = -150;
  else if (angle == 97)    
    angle = 55;
  else if (angle == 122)    
    angle = -55;
#endif*/

  /* Don't know if there is some problem with gyra vs acc 
     But here I look for acc, that might be wrong */
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  return float(ax-angleOffset);
}


/* 
 * Calculate Motor speed and direction
 *
 * Input:  Angle from 0 (+/- ??)
 * Return: Speed for motor (+/- 255)
 */
int calculateMotorSpeed(float angle) {
  float speed = 0;
  
  /* Only calculate integral if angle is big */
  if (angle > SMALL_ERROR || angle < -SMALL_ERROR) {
    integral = integral + angle * DELTA_TIME;
  }
  else {
    integral = 0;
  }

  /* Calculate derivate part */
  derivate = (angle - oldAngle) / DELTA_TIME;

#ifdef DEBUG
  Serial.print("Angle = ");
  Serial.println(angle);
  Serial.print("kp * Angle = ");
  Serial.println(kp * angle);
  Serial.print("Integral = ");
  Serial.println(ki * integral);
  Serial.print("Derivate = ");
  Serial.println(kd * derivate);
  Serial.print("Total speed (float):");
  Serial.println(kp * angle + ki * integral + kd * derivate);
#endif

  speed = (kp *angle + ki * integral + kd * derivate);
  return int(speed);
  
}


/*
 * Set speed of the motors
 *
 * Input:  Speed (+/-255)
 */
void setMotorSpeed(int speed) {
  boolean direction = HIGH;
  /* EXAMPLE */
  
  if (speed <= 0)
    direction = LOW;

#ifdef DEBUG
  Serial.print("Motor speed: ");
  Serial.print(speed);
  Serial.print("   Direction: ");
  Serial.println(direction);
  Serial.print("   Current: ");
  Serial.println(analogRead(CURRENT_A));
#endif

  /* Motor A control */
  digitalWrite(DIR_A, direction);  /* Set direction of motor (HIGH/LOW) */
//  digitalWrite(BRAKE_A, LOW);      /* Disengage the Brake */
  analogWrite(PWM_A, abs(speed));  /* Spins the motor at specific speed */
  
  //Motor B forward @ full speed
//  digitalWrite(13, HIGH); //Establishes forward direction of Channel B
//  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
//  analogWrite(11, 255);   //Spins the motor on Channel B at full speed

//  delay(3000);  
//  digitalWrite(BRAKE_A, HIGH);  //Engage the Brake for Channel A  
//  digitalWrite(DIR_A, HIGH);  //Establishes backward direction of Channel A
//  digitalWrite(BRAKE_A, LOW);   //Disengage the Brake for Channel A
//  analogWrite(PWM_A, speed);    //Spins the motor on Channel A at half speed  

/*  delay(1000);  
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(9, HIGH);  //Engage the Brake for Channel B*/

}
