//
// Created by milutin on 20.8.17..
//

#include "Polynomial.h"
#include "Eigen-3.3/Eigen/Dense"
Polynomial::Polynomial() {

}

void Polynomial::fitPoly(double t0, double x0, double v0, double a0, double t1, double x1, double v1, double a1) {

  Eigen::Matrix<double, 6, 1> rightSide;
  rightSide << x0, v0, a0, x1, v1, a1;
  double t0_2 = t0 * t0, t0_3 = t0_2 * t0, t0_4 = t0_2 * t0_2, t0_5 = t0_4 * t0;
  double t1_2 = t1 * t1, t1_3 = t1_2 * t1, t1_4 = t1_2 * t1_2, t1_5 = t1_4 * t1;
  Eigen::Matrix<double, 6, 6> leftSide;
  leftSide <<
           t0_5, t0_4, t0_3, t0_2, t0, 1,
      5 * t0_4, 4 * t0_3, 3 * t0_2, 2 * t0, 1, 0,
      20 * t0_3, 12 * t0_2, 6 * t0, 2, 0, 0,
      t1_5, t1_4, t1_3, t1_2, t1, 1,
      5 * t1_4, 4 * t1_3, 3 * t1_2, 2 * t1, 1, 0,
      20 * t1_3, 12 * t1_2, 6 * t1, 2, 0, 0;
  coefficients_ = leftSide.colPivHouseholderQr().solve(rightSide);

  //coefficients_ = leftSide.lu().solve(rightSide);
}

double Polynomial::getPosition(double t) {
  double pos = 0;
  for (int i = 0; i < 6; i++)
    pos = pos * t + coefficients_[i];
  return pos;
}

double Polynomial::getVelocity(double t) {
  double vel = 0;
  for (int i = 0; i < 5; i++)
    vel = vel * t + (5 - i) * coefficients_[i];
  return vel;
}

double Polynomial::getAcceleration(double t) {
  double acc = 0;
  for (int i = 0; i < 4; i++)
    acc = acc * t + (5 - i) * (4 - i) * coefficients_[i];
  return acc;
}

double Polynomial::getJerk(double t) {
  double jerk = 0;
  for (int i = 0; i < 3; i++)
    jerk = jerk * t + (5 - i) * (4 - i) * (3-i) * coefficients_[i];
  return jerk;
}
