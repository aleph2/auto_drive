/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <utility>

#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/trajectory.h>
//#include <base_local_planner/trajectory_planner.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <base_local_planner/Position2DInt.h>

#include "wavefront_map_accessor.h"
#include <nav_msgs/Path.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <sparrow_planner/grinding_path.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <sparrow_planner/GetPlan.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
using namespace std;

namespace base_local_planner {

class ScoredTrajectoryTest : public testing::Test {
  public:
    ScoredTrajectoryTest(MapGrid* g, WavefrontMapAccessor* wave, const costmap_2d::Costmap2D& map, std::vector<geometry_msgs::Point> footprint_spec);
    virtual void TestBody(){}

    MapGrid* map_;
    WavefrontMapAccessor* wa;
    CostmapModel cm;
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    base_local_planner::SimpleTrajectoryGenerator generator_;
    base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
    base_local_planner::OscillationCostFunction oscillation_costs_;
    base_local_planner::ObstacleCostFunction obstacle_costs_;
    base_local_planner::MapGridCostFunction path_costs_;
    base_local_planner::MapGridCostFunction goal_costs_;
    base_local_planner::MapGridCostFunction goal_front_costs_;
    base_local_planner::MapGridCostFunction alignment_costs_;
    base_local_planner::TwirlingCostFunction twirling_costs_;
    base_local_planner::LocalPlannerLimits limits;
    base_local_planner::Trajectory result_traj_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    void followPath();
};

ScoredTrajectoryTest::ScoredTrajectoryTest(MapGrid* g, WavefrontMapAccessor* wave, const costmap_2d::Costmap2D& map, std::vector<geometry_msgs::Point> footprint_spec)
: map_(g), wa(wave), cm(map),
obstacle_costs_(wa),
      path_costs_(wa),
      goal_costs_(wa, 0.0, 0.0, true),
      goal_front_costs_(wa, 0.0, 0.0, true),
      alignment_costs_(wa)
{
     generator_.setParameters(
        5.0,
        0.025,
        0.025,
        true,
        0.02); 
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&twirling_costs_); // optionally prefer trajectories that don't spin
    obstacle_costs_.setFootprint(footprint_spec);
    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
      limits.max_trans_vel = 5;
      limits.min_trans_vel = 0;
      limits.max_vel_x = 2;
      limits.min_vel_x = 0.0;
      limits.max_vel_y = 0.0;
      limits.min_vel_y = 0.0;
      limits.max_rot_vel = 5.0;
      limits.min_rot_vel = 0.4;
      limits.acc_lim_x = 1.0;
      limits.acc_lim_y = 0.0;
      limits.acc_lim_theta = 0.5;
      limits.acc_limit_trans = 1.0;
      limits.xy_goal_tolerance = 0.15;
      limits.yaw_goal_tolerance = 0.3;
      limits.prune_plan = false;
      limits.trans_stopped_vel = 1.0;
      limits.rot_stopped_vel = 1.0;

}

void ScoredTrajectoryTest::followPath(){
    Eigen::Vector3f vsamples_;
    vsamples_[0] = 5;
    vsamples_[1] = 1;
    vsamples_[2] = 5;
    Eigen::Vector3f pos(1.0, 1.0, 0);
    Eigen::Vector3f vel(1.0, 0, 0);
    Eigen::Vector3f goal(2.0, 3.0, 1);
      geometry_msgs::PoseStamped start;
  start.pose.position.x = 1;
  start.pose.position.y = 1;
  geometry_msgs::PoseStamped end;
  end.pose.position.x = 3;
  end.pose.position.y = 3;
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<sparrow_planner::GetPlan>("path_test_service");
  sparrow_planner::GetPlan srv;
  if(client.call(srv)){
    nav_msgs::Path path = srv.response.plan;
    global_plan_.clear();
    global_plan_.insert(global_plan_.begin(), path.poses.begin(), path.poses.end());
    ROS_DEBUG_STREAM("Path pose size is " << path.poses.size());
  }else {
    return;
  } 
   
  path_costs_.setTargetPoses(global_plan_);
  goal_costs_.setTargetPoses(global_plan_);
  goal_front_costs_.setTargetPoses(global_plan_);
  alignment_costs_.setTargetPoses(global_plan_);
  wa->synchronize();
  ROS_DEBUG_STREAM("The end point of plan is " << global_plan_[1]);
  for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics.begin(); loop_critic != critics.end(); ++loop_critic) {
    TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
      }
    }
    generator_.initialise(pos,
        vel,
        global_plan_.back(),
        &limits,
        vsamples_);
    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
    double x, y, z;
//    ROS_DEBUG_STREAM("Velocity is:" << result_traj_.xv_ << " " << result_traj_.yv_ << " " << result_traj_.thetav_); 
    for(int i = 0; i < all_explored.size(); i++){
        Trajectory t = all_explored[i];
        ROS_DEBUG_STREAM("Velocity is:" << t.xv_ << " " << t.yv_ << " " << t.thetav_);
    }
    Trajectory t = result_traj_;
   for( int i = 0; i < t.getPointsSize(); i++) {
       t.getPoint(i, x, y, z);
       ROS_DEBUG_STREAM("Point is:" << x << " " << y << " " << z); 
    }
       
}


ScoredTrajectoryTest* sct = NULL;

ScoredTrajectoryTest* setup_scoretestclass_singleton() {
  if (sct == NULL) {
    MapGrid* mg = new MapGrid (10, 10);
    WavefrontMapAccessor* wa = new WavefrontMapAccessor(mg, 5);
    const costmap_2d::Costmap2D& map = *wa;
    std::vector<geometry_msgs::Point> footprint_spec;
    geometry_msgs::Point pt;
    //create a square footprint
    pt.x = 1;
    pt.y = 1;
    footprint_spec.push_back(pt);
    pt.x = 1;
    pt.y = 0;
    footprint_spec.push_back(pt);
    pt.x = 0;
    pt.y = 0;
    footprint_spec.push_back(pt);
    pt.x = 0;
    pt.y = 1;
    footprint_spec.push_back(pt);

    sct = new base_local_planner::ScoredTrajectoryTest(mg, wa, map, footprint_spec);
  }
  return sct;
}

//make sure that trajectories that intersect obstacles are invalidated
TEST(ScoredTrajectoryTest, followPath){
  ScoredTrajectoryTest* sct = setup_scoretestclass_singleton();
  sct->followPath();
}

}; //namespace

int main( int argc, char **argv ) {
  ros::init(argc, argv, "add_two_ints_client");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
