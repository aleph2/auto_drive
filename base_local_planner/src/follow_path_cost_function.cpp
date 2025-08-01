/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <base_local_planner/follow_path_cost_function.h>

#include <math.h>

namespace base_local_planner {


double FollowPathCostFunction::scoreTrajectory(Trajectory &traj) {
  // backward motions bad on a robot without backward sensors
/*  if (traj.xv_ < 0.0) {
    return penalty_;
  }
  // strafing motions also bad on such a robot
  if (traj.xv_ < 0.1 && fabs(traj.thetav_) < 0.2) {
    return penalty_;
  }
  // the more we rotate, the less we progress forward
  return fabs(traj.thetav_) * 10;
*/
  return 0.0;
}

void FollowPathCostFunction::setPath(std::vector<geometry_msgs::PoseStamped> &transformed_path) {
  
}
} /* namespace base_local_planner */
