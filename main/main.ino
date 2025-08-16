#include <cmath>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//determined experimentally
#define MG996R_MIN 140  // pulse length count for 0°
#define MG996R_MID 345  // pulse length count for 90°
#define MG996R_MAX 560  // pulse length count for 180°

#define MG90S_MIN 100
#define MG90S_MID 315
#define MG90S_MAX 520

//PWM channels on PCA9685
#define j1 0
#define j2 1

float a1, a2;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz standard for hobby servos
}

int angleToPulse(int angle, char type) {
  if (type == 'r') return map(angle, 0, 180, MG996R_MIN, MG996R_MAX);
  return map(angle, 0, 180, MG90S_MIN, MG90S, MAX);
}

void setPos(float x, float y) {

  // geometrically derived inverse kinematics of 2d planar joint
  float r, alpha, beta, gamma, q1, q2;
  // a1,a2 = link lengths
  //q1, q2 = joint angles
  // x, y = cartesian coordinates of end effector
  r = Math.sqrt(pow(x, 2) + pow(y, 2));

  alpha = acos((pow(a1, 2) + pow(a2, 2) - pow(r, 2)) / (2 * a1 * a2));

  q2 = pi - alpha;

  gamma = atan(y / x);

  beta = atan((a2 * sin(q2)) / (a1 + a2 * cos(q2)));

  q1 = gamma - beta;

  pwm.setPWM(j1, 0, angleToPulse(q1, 's'));
  pwm.setPWM(j2, 0, angleToPulse(q2, 's'));
}

void loop() {
}
