#include <cmath>

float a1, a2;

void setup() {

}

// geometric inverse kinematics of 2d planar joint
void setPos(float x, float y) {

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

}

void loop() {
}
