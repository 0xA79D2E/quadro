#include<cmath>

void setup() {

// inverse kinematics of 2d planar joint

float x, y, r, alpha, beta, gamma, a1, a2, q1, q2;
// a1,a2 = link lengths
//q1, q2 = joint angles
// x, y = cartesian coordinates of end effector
r = Math.sqrt(pow(x, 2) + pow(y, 2));

alpha = acos( (pow(a1,2) + pow(a2,2) - pow(r,2)) / (2*a1*a2) );

q2 = pi - alpha;

gamma = atan(y/x);

beta = atan( (a2*sin(q2) ) / (a1 + a2*cos(q2)) );

q1 = gamma - beta; //case where q2 > 0

// q1 = gamma + beta; //case where q2 < 0


}

void loop() {

}
