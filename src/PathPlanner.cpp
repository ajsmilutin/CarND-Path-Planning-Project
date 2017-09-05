//
// Created by milutin on 20.8.17..
//

#include "PathPlanner.h"
#include "PolynomialXY.h"
#include <iostream>

void PathPlanner::setCarPosition(Eigen::VectorXd &carState) {
  carState_ = carState;
  carState_[SPEED] *= 1.6 / 3.6;
}


void PathPlanner::SetPathHelper(PathHelpers &ph) {
  path_helper_ = ph;
}

double PathPlanner::getX() {
  return carState_[COORDINATE_X];
}

double PathPlanner::getY() {
  return carState_[COORDINATE_Y];
}

double PathPlanner::getS() {
  return carState_[COORDINATE_S];
}

double PathPlanner::getD() {
  return carState_[COORDINATE_D];
}

double PathPlanner::getYaw() {
  return carState_[COORDINATE_YAW];
}

int PathPlanner::getPrevLength() {
  return prev_x_.size();
}

vector<double> PathPlanner::getLastXY() {
  int size = prev_y_.size();
  double theta = atan2(prev_y_[size - 1] - prev_y_[size - 2], prev_x_[size - 1] - prev_x_[size - 2]);
  return {prev_x_[size - 1], prev_y_[size - 1], theta};
}

vector<double> PathPlanner::getLastXYVel() {
  int size = prev_y_.size();
  double vx = (prev_x_[size - 1] - prev_x_[size - 2]) / 0.02;
  double vy = (prev_y_[size - 1] - prev_y_[size - 2]) / 0.02;
  return {vx, vy};
}

vector<double> PathPlanner::getLastXYAcc() {
  int size = prev_y_.size();
  double vx = (prev_x_[size - 1] - prev_x_[size - 2]) / 0.02;
  double vy = (prev_y_[size - 1] - prev_y_[size - 2]) / 0.02;
  double vx1 = (prev_x_[size - 2] - prev_x_[size - 3]) / 0.02;
  double vy1 = (prev_y_[size - 2] - prev_y_[size - 3]) / 0.02;
  return {(vx - vx1) / 0.02, (vy - vy1) / 0.02};
}

vector<double> PathPlanner::getLastSD() {
  vector<double> xytheta = getLastXY();
  return path_helper_.getFrenet(xytheta[0], xytheta[1], xytheta[2]);
}

vector<double> PathPlanner::getLastSDVel() {
  vector<double> xytheta = getLastXY();
  vector<double> xyvel = getLastXYVel();
  return path_helper_.getSDVelocity(xytheta[0], xytheta[1], xytheta[2], xyvel[0], xyvel[1]);
}

vector<double> PathPlanner::getLastSDAcc() {
  vector<double> xytheta = getLastXY();
  vector<double> xyacc = getLastXYAcc();
  return path_helper_.getSDAcceleration(xytheta[0], xytheta[1], xytheta[2], xyacc[0], xyacc[1]);
}

void PathPlanner::setFusionData(const FusionData &data) {
  if (prev_x_.size() <= 3) return;

  map<int, double> target_vel;
  map<int, double> min_vel;
  target_vel[0] = max_vel_;
  min_vel[0] = 0;
  target_vel[1] = max_vel_;
  min_vel[1] = 0;
  target_vel[2] = max_vel_;
  min_vel[2] = 0;

  vector<double> car_sd = getLastSD();
  vector<double> car_sd_vel = getLastSDVel();

  for (auto it = data.begin(); it != data.end(); it++) {
    int id = it->first;
    double x = it->second[0];
    double y = it->second[1];
    double vx = it->second[2];
    double vy = it->second[3];
    double s = it->second[4];
    double d = it->second[5];

    // collision from front
    vector<double> sdvel = path_helper_.getSDVelocity(x, y, atan2(vy, vx), vx, vy);
    s += sdvel[0] * 0.02 * getPrevLength();
    int lane = floor(d / 4.0);
    double car_length = 16;

    for (int i = 1; i <= 8; i++) {
      double mul = 0.25 * i;
      if (car_sd[0] + (target_vel[lane] + car_sd_vel[0])* 0.5 * mul * time_horizon_ + car_length > s + sdvel[0] * mul * time_horizon_ &&
          s  > car_sd[0])
        target_vel[lane] = (s + sdvel[0] * mul * time_horizon_ - car_length - car_sd[0]) * 2.0 / mul / time_horizon_ - car_sd_vel[0];

      if (s + sdvel[0] * mul * time_horizon_ + car_length > car_sd[0] + (min_vel[lane] + car_sd_vel[0])*0.5 * mul * time_horizon_ &&
          s <= car_sd[0] )
        min_vel[lane] = (s + sdvel[0] * mul * time_horizon_ + car_length - car_sd[0]) * 2.0 / mul / time_horizon_ - car_sd_vel[0];
    }
  }

  for (int new_lane = 0; new_lane < 3; new_lane++) {
    if (abs(new_lane - lane_) >= 2) continue;
    if (target_vel[new_lane] < min_vel[new_lane] + 2 ) continue;
    if (target_vel[new_lane] > target_vel[lane_] + 2) // lane change
      setLane(new_lane);
  }
  target_vel_ = target_vel[lane_];
  target_vel_ = max(0.0, target_vel_);
}

void PathPlanner::setPreviousPath(const std::vector<double> &prev_x, const std::vector<double> &prev_y) {
  int items_remaining = prev_x.size();
  // use previously stored next_x_
  // since ones published by the simulator are too coarse
  prev_x_.clear();
  prev_y_.clear();

  if (items_remaining <= next_x_.size()) {
    int next_size = next_x_.size();
    for (int i = next_size - items_remaining; i < min(next_size, next_size - items_remaining + 5); i++) {
      prev_x_.push_back(next_x_[i]);
      prev_y_.push_back(next_y_[i]);
    }
  }

}

void PathPlanner::nextXY(std::vector<double> &next_x, std::vector<double> &next_y) {
  for (int i = 0; i < prev_x_.size(); i++) {
    next_x.push_back(prev_x_[i]);
    next_y.push_back(prev_y_[i]);
  }

  if (next_x.size() > 20) return;
  if (next_x.size() < 3) {
    double x = getX();
    double y = getY();

    for (int i = 0; i < 10; i++) {
      next_x.push_back(x);
      next_y.push_back(y);
    }
    next_x_ = next_x;
    next_y_ = next_y;
    return;
  }

  vector<double> last_xy = getLastXY();
  vector<double> sd = getLastSD();
  vector<double> sdvel = getLastSDVel();
  vector<double> sdacc = getLastSDAcc();

  double s1 = 0;
  double vs1 = 0;
  double as1 = 0;

  double time_to_max = (target_vel_ - sdvel[0]) / max_acc_;
  double acc = max_acc_;
  if (time_to_max < 0) {
    acc = -acc;
    time_to_max = abs(time_to_max);
  }

  if (time_to_max > time_horizon_) {
    s1 = sd[0] + sdvel[0] * time_horizon_ + acc * 0.5 * time_horizon_ * time_horizon_;
    vs1 = sdvel[0] + acc * time_horizon_;
    as1 = acc;
  } else {
    acc = (target_vel_ - sdvel[0]) / time_horizon_;
    s1 = sd[0] + sdvel[0] * time_horizon_ + acc * time_horizon_ * time_horizon_ * 0.5;
    vs1 = target_vel_;
    as1 = 0;
  }

  double d1 = 2 + lane_ * 4;
  double vd1 = 0;
  double ad1 = 0;

  double dvel = 4 / time_horizon_;
  double time_change = (d1 - sd[1]) / dvel;

  if (time_change < 0) {
    dvel = -dvel;
    time_change = abs(time_change);
  }

  if (time_change > time_horizon_) {
    d1 = sd[1] + dvel * time_horizon_;
    vd1 = dvel;
  } else {

  }


  vector<double> pos1 = path_helper_.getXY(s1, d1);
  vector<double> v0 = getLastXYVel();
  vector<double> v1 = path_helper_.getXYVel(s1, d1, vs1, vd1);
  vector<double> a0 = getLastXYAcc();
  vector<double> a1 = path_helper_.getXYVel(s1, d1, as1, ad1);

  PolynomialXY new_path;

  new_path.fitPolyXY(0, last_xy[0], last_xy[1], v0[0], v0[1], a0[0], a0[1],
                     time_horizon_, pos1[0], pos1[1], v1[0], v1[1], a1[0], a1[1]);


  new_path.evaluate_safe(0, 0.02, time_horizon_ / 0.02, 8, 8, 21.5, next_x, next_y, next_x.size());


  next_x_ = next_x;
  next_y_ = next_y;
}

PathPlanner::PathPlanner(double max_vel, double max_acc, double th)
    : path_helper_(), carState_(6), max_vel_(max_vel), target_vel_(max_vel), max_acc_(max_acc), time_horizon_(th) {

}

