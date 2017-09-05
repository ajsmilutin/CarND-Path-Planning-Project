//
// Created by milutin on 20.8.17..
//

#include <iostream>
#include "PathHelpers.h"

void PathHelpers::push_back(double x, double y, double s, double d_x, double d_y){
  maps_x_.push_back(x);
  maps_y_.push_back(y);
  maps_s_.push_back(s);
  maps_d_x_.push_back(d_x);
  maps_d_y_.push_back(d_y);

}

int PathHelpers::ClosestWaypoint(double x, double y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x_.size(); i++) {
    double map_x = maps_x_[i];
    double map_y = maps_y_[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}

int PathHelpers::NextWaypoint(double x, double y, double theta) {

  int closestWaypoint = ClosestWaypoint(x, y);

  double map_x = maps_x_[closestWaypoint];
  double map_y = maps_y_[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}

vector<double> PathHelpers::getFrenet(double x, double y, double theta) {
  int next_wp = NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x_.size() - 1;
  }

  double n_x = maps_x_[next_wp] - maps_x_[prev_wp];
  double n_y = maps_y_[next_wp] - maps_y_[prev_wp];
  double x_x = x - maps_x_[prev_wp];
  double x_y = y - maps_y_[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x_[prev_wp];
  double center_y = 2000 - maps_y_[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x_[i], maps_y_[i], maps_x_[i + 1], maps_y_[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

vector<double> PathHelpers::getXY(double s, double d)  {
  int prev_wp = -1;

  while (s > maps_s_[prev_wp + 1] && (prev_wp < (int) (maps_s_.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x_.size();

  double heading = atan2((maps_y_[wp2] - maps_y_[prev_wp]), (maps_x_[wp2] - maps_x_[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s_[prev_wp]);

  double seg_x = maps_x_[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y_[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

vector<double> PathHelpers::getXYVel(double s, double d, double svel, double dvel)  {
  int prev_wp = -1;

  while (s > maps_s_[prev_wp + 1] && (prev_wp < (int) (maps_s_.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x_.size();

  double heading = atan2((maps_y_[wp2] - maps_y_[prev_wp]), (maps_x_[wp2] - maps_x_[prev_wp]));
  // the x,y,s along the segment
  double perp_heading = heading - pi() / 2;


  double x_vel = svel*cos(heading) +  dvel * cos(perp_heading);
  double y_vel = svel*sin(heading) +  dvel * sin(perp_heading);

  return {x_vel, y_vel};
}


vector<double> PathHelpers::getSDVelocity(double x, double y, double theta, double vx, double vy) {
  return getSDAcceleration(x, y, theta, vx, vy);
}

void PathHelpers::GetDirection(double x, double y, double theta, double &n_x, double &n_y) {
  int next_wp = NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x_.size() - 1;
  }

  n_x = maps_x_[next_wp] - maps_x_[prev_wp]; //direction of s coordinate
  n_y = maps_y_[next_wp] - maps_y_[prev_wp];
}

vector<double> PathHelpers::getSDAcceleration(double x, double y, double theta, double ax, double ay) {
  double dir_sx;
  double dir_sy;
  double dir_dx;
  double dir_dy;
  GetDirection(x,y,theta, dir_sx, dir_sy);

  double n_intensity = sqrt(dir_sx*dir_sx + dir_sy*dir_sy);
  dir_sx/=n_intensity;
  dir_sy/=n_intensity;

  dir_dy = -dir_sx;
  dir_dx = dir_sy;
  return {ax*dir_sx+ ay*dir_sy, ax*dir_dx, ax*dir_dy};
}

