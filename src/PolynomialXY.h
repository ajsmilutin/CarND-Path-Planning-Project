//
// Created by milutin on 20.8.17..
//

#ifndef PATH_PLANNING_POLYNOMIALXY_H
#define PATH_PLANNING_POLYNOMIALXY_H

#include "Polynomial.h"
#include <vector>
class PolynomialXY {
  Polynomial polyX_;
  Polynomial polyY_;

public:
  void fitPolyXY(double t0, double x0, double y0, double vx0, double vy0, double ax0, double ay0,
                 double t1, double x1, double y1, double vx1, double vy1, double ax1, double ay1);
  void evaluate(double t0, double dt, int n_samples, std::vector<double>& next_x, std::vector<double>& next_y, int start_from =0);
  void evaluate_safe(double t0, double dt, int n_samples, double max_jerk, double max_acc, double max_vel,  std::vector<double>& next_x, std::vector<double>& next_y, int start_from =0);

  double getTotalVelocity(double t0);
  double getTotalAcceleration(double t0);
  double getTotalJerk(double t0);
};


#endif //PATH_PLANNING_POLYNOMIALXY_H
