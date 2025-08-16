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

//testing lengths
float a1 = 12;
float a2 = 10;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz standard for hobby servos
  Serial.begin(9600);
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

  // algebraically simplified with MATLAB
  float angle1 = atan(y/x) - atan((2*a1*a2*(1 - (a1^2 + a2^2 - x^2 - y^2)^2/(4*a1^2*a2^2))^(1/2))/(a1^2 - a2^2 + x^2 + y^2)) 
  float angle2 = pi - acos((a1^2 + a2^2 - x^2 - y^2)/(2*a1*a2)); 

  pwm.setPWM(j1, 0, angleToPulse(angle1, 's'));
  pwm.setPWM(j2, 0, angleToPulse(angle2, 's'));
  Serial.print("Angle1: ");
  Serial.print(angle1);
  Serial.print( "Angle2: ");
  Serial.println(angle2);
}

void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim;

    int separator = input.indexOf(':');
    if (separator > 0) {
      String xStr = input.substring(0,separator);
      String yStr = input.substring(separator + 1);
      int x = xStr.toFloat();
      int y = yStr.toFloat();
      setPos(float x, float y);
      Serial.print("X: ");
      Serial.print(x);
      Serial.print(" Y: ");
      Serial.println(y);
    }
  }
}
