//
// Created by milutin on 20.8.17..
//

#ifndef PATH_PLANNING_STATEMACHINE_H
#define PATH_PLANNING_STATEMACHINE_H

#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "PathHelpers.h"
#include <vector>

typedef std::map<int, vector<double>> FusionData;
class PathPlanner {

  Eigen::VectorXd carState_;
  PathHelpers path_helper_;
  enum CarStates {COORDINATE_X, COORDINATE_Y, COORDINATE_S, COORDINATE_D, COORDINATE_YAW, SPEED};
  std::vector<double> prev_x_;
  std::vector<double> prev_y_;
  std::vector<double> next_x_;
  std::vector<double> next_y_;

  double target_vel_;
  double max_vel_;
  double max_acc_;
  double time_horizon_;
  int lane_;


public:
  PathPlanner(double max_vel, double max_acc, double time_horizon_=1.5);
  void SetPathHelper(PathHelpers&);
  void setCarPosition(Eigen::VectorXd& cs);
  void setFusionData(const FusionData&);
  void setPreviousPath(const std::vector<double>& prev_x,const std::vector<double>& prev_y);
  void nextXY(std::vector<double>& next_x, std::vector<double>& next_y);

  void setLane(int lane){lane_ = lane;};
  vector<double> getLastXY();
  vector<double> getLastXYVel();
  vector<double> getLastXYAcc();

  vector<double> getLastSD();
  vector<double> getLastSDVel();
  vector<double> getLastSDAcc();

  double getX();
  double getY();
  double getS();
  double getD();
  double getYaw();
  int    getPrevLength();
};


#endif //PATH_PLANNING_STATEMACHINE_H
