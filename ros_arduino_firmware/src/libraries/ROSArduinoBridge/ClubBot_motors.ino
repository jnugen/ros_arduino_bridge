//  This example code is in the public domain.

 
// Pin definitions:
static const unsigned char left_motor_enable = 10;
static const unsigned char left_motor_a = 9;
static const unsigned char left_motor_b = 8;
static const unsigned char right_motor_enable = 3;
static const unsigned char right_motor_a = 4;
static const unsigned char right_motor_b = 5;

// the init routine does the setup for the robot I/O pins
void clubbot_initMotors(void) {
  // Set H-bridge control pins:
  pinMode(left_motor_enable, OUTPUT);
  pinMode(left_motor_a, OUTPUT);
  pinMode(left_motor_b, OUTPUT);
  pinMode(right_motor_enable, OUTPUT);
  pinMode(right_motor_a, OUTPUT);
  pinMode(right_motor_b, OUTPUT);
  
  // Set all the pin values to predefined values:
  digitalWrite(left_motor_enable, LOW);
  digitalWrite(left_motor_a, LOW);
  digitalWrite(left_motor_b, LOW);
  digitalWrite(right_motor_enable, LOW);
  digitalWrite(right_motor_a, LOW);
  digitalWrite(right_motor_b, LOW);
}

void clubbot_setSpeedLeft(int speed) {
  // Set left motor direction:
  if (speed > 0) {
    // Set left motor to go forward:
    digitalWrite(left_motor_a, HIGH);
    digitalWrite(left_motor_b, LOW);
  } else {
    // Set left motor to go backward:
    digitalWrite(left_motor_a, LOW);
    digitalWrite(left_motor_b, HIGH);
    speed = -speed;                   // Make left a positive value:
  }

  // Start motor
  analogWrite(left_motor_enable, speed);
}

void clubbot_setSpeedRight(int speed) {
  // Set right motor direction:
  if (speed > 0) {
    // Set right motor to go forward:
    digitalWrite(right_motor_a, HIGH);
    digitalWrite(right_motor_b, LOW);
  } else {
    // Set right motor to go backward:
    digitalWrite(right_motor_a, LOW);
    digitalWrite(right_motor_b, HIGH);
    speed = -speed;                 // Make right a positive value:
  }

  // Start motor
  analogWrite(right_motor_enable, speed);
}

