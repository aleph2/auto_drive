#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <string>
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
//#include <costmap_2d/TrajectoryConstraintsPlugInConfig.h>
#include <simple_layers/TrajectoryConstraintsPlugInConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sparrow_planner/points.h>
namespace simple_layer_namespace
{

class SimpleLayer : public costmap_2d::Layer
{
public:
  SimpleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  void constrCb(const sparrow_planner::points& points);
private:
  void reconfigureCB(costmap_2d::TrajectoryConstraintsPlugInConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::TrajectoryConstraintsPlugInConfig> *dsrv_;
  std::string frame_id_;;  
  std::string global_frame_;
  ros::Subscriber constr_sub_;
  bool constr_published_;
  std::vector<tf::Point> constr_points_;
  void updateCellCost(costmap_2d::Costmap2D& master_grid, tf::Point, const tf::StampedTransform transform);
};
}
#endif
