#ifndef FOLLOW_PATH_COST_FUNCTION_H_
#define FOLLOW_PATH_COST_FUNCTION_H_
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace base_local_planner {

class FollowPathCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

//  FollowPathCostFunction(double penalty) : penalty_(penalty) {}
  FollowPathCostFunction() {}
  ~FollowPathCostFunction() {}

  double scoreTrajectory(Trajectory &traj);
  
  void setPath(std::vector<geometry_msgs::PoseStamped> &transformed_path);
  bool prepare() {return true;};

private:
  std::vector<Eigen::Vector3f> path_;
};

} /* namespace base_local_planner */
#endif /* PREFER_FORWARD_COST_FUNCTION_H_ */
