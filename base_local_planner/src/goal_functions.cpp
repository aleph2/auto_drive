/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <base_local_planner/goal_functions.h>

namespace base_local_planner {

  double getGoalPositionDistance(const tf::Stamped<tf::Pose>& global_pose, double goal_x, double goal_y) {
    return hypot(goal_x - global_pose.getOrigin().x(), goal_y - global_pose.getOrigin().y());
  }

  double getGoalOrientationAngleDifference(const tf::Stamped<tf::Pose>& global_pose, double goal_th) {
    double yaw = tf::getYaw(global_pose.getRotation());
    return angles::shortest_angular_distance(yaw, goal_th);
  }

  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {
    //given an empty path we won't do anything
    if(path.empty())
      return;

    //create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
  }

  void prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan){
    ROS_ASSERT(global_plan.size() >= plan.size());
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator global_it = global_plan.begin();
    while(it != plan.end()){
      const geometry_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.getOrigin().x() - w.pose.position.x;
      double y_diff = global_pose.getOrigin().y() - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }

  bool transformGlobalPlan(
      const tf::TransformListener& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const tf::Stamped<tf::Pose>& global_pose,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    transformed_plan.clear();

    if (global_plan.empty()) {
      ROS_ERROR("Received plan with zero length");
      return false;
    }

    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    try {
      // get plan_to_global_transform from plan frame to global_frame
      tf::StampedTransform plan_to_global_transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_pose.header.frame_id, plan_pose.header.stamp,
                          plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_pose.header.frame_id, plan_pose.header.stamp, 
                         plan_pose.header.frame_id, plan_to_global_transform);

      //let's get the pose of the robot in the frame of the plan
      tf::Stamped<tf::Pose> robot_pose;
      tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

      unsigned int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 0;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      while(i < (unsigned int)global_plan.size()) {
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        if (sq_dist <= sq_dist_threshold) {
          break;
        }
        ++i;
      }

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      //now we'll transform until points are outside of our distance threshold
      while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        ++i;
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  /***
   for sparrow planner, we need to change the behavior of tailor plan in local costmap area
   Logic is: based on current pose and current path idx to determine if we are on the line of 
   local path.  

  */
  bool transformSparrowGlobalPlan(
      const tf::TransformListener& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::vector<int>& path_idx,
      int& current_path_idx,
      const tf::Stamped<tf::Pose>& global_pose,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    transformed_plan.clear();

    if (global_plan.empty()) {
      ROS_ERROR("Received plan with zero length");
      return false;
    }

    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    try {
      // get plan_to_global_transform from plan frame to global_frame
      tf::StampedTransform plan_to_global_transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_pose.header.frame_id, plan_pose.header.stamp,
                          plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_pose.header.frame_id, plan_pose.header.stamp, 
                         plan_pose.header.frame_id, plan_to_global_transform);

      //let's get the pose of the robot in the frame of the plan
      tf::Stamped<tf::Pose> robot_pose;
      tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

      //we'll discard points on the plan that are outside the local costmap

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      int start_idx = path_idx[current_path_idx];
      int end_idx = current_path_idx < global_plan.size()?path_idx[current_path_idx + 1]:global_plan.size() - 1; 
      for(int k = start_idx; k <= end_idx; k++) {
        const geometry_msgs::PoseStamped& pose = global_plan[k];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);
        transformed_plan.push_back(newer_pose);
      }
      tf::Stamped<tf::Pose> goal_pose; 
      std::vector<geometry_msgs::PoseStamped> tmp_plan;
      tmp_plan.push_back(global_plan[end_idx]);
      getGoalPose(tf,tmp_plan, global_frame, goal_pose); 
      double goal_x = goal_pose.getOrigin().getX();

      double goal_y = goal_pose.getOrigin().getY();
      if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= 0.1 && current_path_idx < path_idx.size() - 1) {
        //check to see if the goal orientation has been reached
        current_path_idx++;
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  bool getGoalPose(const tf::TransformListener& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame, tf::Stamped<tf::Pose>& goal_pose) {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return false;
    }

    const geometry_msgs::PoseStamped& plan_goal_pose = global_plan.back();
    try{
      tf::StampedTransform transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                          plan_goal_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                         plan_goal_pose.header.frame_id, transform);

      poseStampedMsgToTF(plan_goal_pose, goal_pose);
      goal_pose.setData(transform * goal_pose);
      goal_pose.stamp_ = transform.stamp_;
      goal_pose.frame_id_ = global_frame;

    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }
    return true;
  }

  bool isGoalReached(const tf::TransformListener& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const costmap_2d::Costmap2D& costmap __attribute__((unused)),
      const std::string& global_frame,
      tf::Stamped<tf::Pose>& global_pose,
      const nav_msgs::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance){

    //we assume the global goal is the last point in the global plan
    tf::Stamped<tf::Pose> goal_pose;
    getGoalPose(tf, global_plan, global_frame, goal_pose);

    double goal_x = goal_pose.getOrigin().getX();
    double goal_y = goal_pose.getOrigin().getY();
    double goal_th = tf::getYaw(goal_pose.getRotation());

    //check to see if we've reached the goal position
    if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
      //check to see if the goal orientation has been reached
      if(fabs(getGoalOrientationAngleDifference(global_pose, goal_th)) <= yaw_goal_tolerance) {
        //make sure that we're actually stopped before returning success
        if(stopped(base_odom, rot_stopped_vel, trans_stopped_vel))
          return true;
      }
    }

    return false;
  }

  bool stopped(const nav_msgs::Odometry& base_odom, 
      const double& rot_stopped_velocity, const double& trans_stopped_velocity){
    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity 
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity;
  }
};
