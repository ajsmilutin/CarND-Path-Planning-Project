//
// Created by milutin on 20.8.17..
//

#ifndef PATH_PLANNING_POLYNOMIAL_H
#define PATH_PLANNING_POLYNOMIAL_H

#include "Eigen-3.3/Eigen/Core"

class Polynomial {
  Eigen::Matrix<double, 6, 1> coefficients_;

public:
  Polynomial();
  void fitPoly(double t0, double x0, double v0, double a0, double t1, double x1, double v1, double a1);
  double getPosition(double t);
  double getVelocity(double t);
  double getAcceleration(double t);
  double getJerk(double t);

};


#endif //PATH_PLANNING_POLYNOMIAL_H
