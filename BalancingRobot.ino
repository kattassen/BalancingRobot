/*
  Selfbalancing Robot  
 
 Created 8 Aug 2012
 Author: Simon Furborg
 
 */

/* Debug flag */
#define DEBUG 1

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

float robotAngle = 0;
float oldAngle = 0;



void setup() {
  /* Initiate some modules */

  /* Setup the motor shield */
  /* Setup Motor Channel A */
  pinMode(DIR_A, OUTPUT);    /* Initiates Motor Channel A pin */
  pinMode(BRAKE_A, OUTPUT);  /* Initiates Brake Channel A pin */
  /* Setup Motor Channel B */
  pinMode(DIR_B, OUTPUT);    /* Initiates Motor Channel B pin */
  pinMode(BRAKE_B, OUTPUT);  /* Initiates Brake Channel B pin */

  /* Initiate debug mode (serial printouts) */
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Hello there, let's start!");
#endif


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
int getAngle() {
  int angle = 0;

  /* Get "angle" from potentiometer */
#ifdef DEBUG    
  angle = (analogRead(2) - 500) /3;
#endif
  
  return float(angle);
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
