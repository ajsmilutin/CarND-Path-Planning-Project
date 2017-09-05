//
// Created by milutin on 20.8.17..
//

#ifndef PATH_PLANNING_PATHHELPERS_H
#define PATH_PLANNING_PATHHELPERS_H


#include <math.h>
#include <vector>
#include <cmath>

using namespace std;

class PathHelpers {
  vector<double> maps_x_;
  vector<double> maps_y_;
  vector<double> maps_s_;
  vector<double> maps_d_x_;
  vector<double> maps_d_y_;
  void GetDirection(double x, double y, double theta, double& n_x, double& n_y);

public:
  PathHelpers(){};
  static constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }
  double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  void push_back(double x, double y, double s, double d_x, double d_y);

  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta);
// Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d);
  vector<double> getXYVel(double s, double d, double svel, double dvel);

  // get velocity of s coordinate
  vector<double> getSDVelocity(double x, double y, double theta, double vx, double vy);
  vector<double> getSDAcceleration(double x, double y, double theta,  double ax, double ay);
};


#endif //PATH_PLANNING_PATHHELPERS_H
