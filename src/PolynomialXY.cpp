//
// Created by milutin on 20.8.17..
//

#include "PolynomialXY.h"
#include <iostream>

double min(double a, double b){
  if (a<b) return a;
  return b;
}
void PolynomialXY::fitPolyXY(double t0, double x0, double y0, double vx0, double vy0, double ax0, double ay0,
                             double t1, double x1, double y1, double vx1, double vy1, double ax1, double ay1) {
  polyX_.fitPoly(t0, x0, vx0, ax0, t1, x1, vx1, ax1);
  polyY_.fitPoly(t0, y0, vy0, ay0, t1, y1, vy1, ay1);
}

void
PolynomialXY::evaluate(double t0, double dt, int n_samples, std::vector<double> &next_x, std::vector<double> &next_y, int start_from) {
  double t = t0;
  for (int i = start_from; i < n_samples; i++) {
    next_x.push_back(polyX_.getPosition(t));
    next_y.push_back(polyY_.getPosition(t));
    t += dt;
  }

}

double PolynomialXY::getTotalVelocity(double t0) {
  double xvel=polyX_.getVelocity(t0);
  double yvel=polyY_.getVelocity(t0);
  return sqrt(xvel*xvel + yvel*yvel);
}

double PolynomialXY::getTotalAcceleration(double t0) {
  double xacc = polyX_.getAcceleration(t0);
  double yacc = polyY_.getAcceleration(t0);
  return sqrt(xacc*xacc + yacc*yacc);

}

double PolynomialXY::getTotalJerk(double t0) {
  double xjerk = polyX_.getJerk(t0);
  double yjerk = polyY_.getJerk(t0);
  return sqrt(xjerk*xjerk + yjerk*yjerk);
}

void PolynomialXY::evaluate_safe(double t0, double dt, int n_samples, double max_jerk, double max_acc, double max_vel,
                                 std::vector<double> &next_x, std::vector<double> &next_y, int start_from) {
  double t=t0;
  double px=polyX_.getPosition(t0);
  double py=polyY_.getPosition(t0);
  double vx=polyX_.getVelocity(t0);
  double vy=polyY_.getVelocity(t0);
  double ax=polyX_.getAcceleration(t0);
  double ay=polyY_.getAcceleration(t0);

  dt = dt/2;
  for (int i=start_from; i<n_samples; i++){
    for (int j=0; j<2; j++) {
      ax = polyX_.getAcceleration(t);
      ay = polyY_.getAcceleration(t);

      double acc_scale = min(1, max_acc / sqrt(ax * ax + ay * ay));
      ax *= acc_scale;
      ay *= acc_scale;

      vx += ax * dt;
      vy += ay * dt;
      double vel_scale = min(1, max_vel / sqrt(vx * vx + vy * vy));
      vx *= vel_scale;
      vy *= vel_scale;

      px += vx * dt;
      py += vy * dt;
      t+=dt;
    }
    next_x.push_back(px);
    next_y.push_back(py);


  }
}
