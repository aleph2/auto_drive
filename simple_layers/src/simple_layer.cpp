#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer():constr_published_(false), constr_points_(0) {

}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::TrajectoryConstraintsPlugInConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::TrajectoryConstraintsPlugInConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();
  constr_sub_ = nh.subscribe("/move_base/SparrowPlanner/constrs_points", 10, &SimpleLayer::constrCb, this);
}


void SimpleLayer::reconfigureCB(costmap_2d::TrajectoryConstraintsPlugInConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  frame_id_ = config.frame_id;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  ROS_INFO_STREAM("The bound is :" << *min_x << "," << *min_y << "," << *max_x << "," << *max_y);
  if (!enabled_)
    return;
  mark_x_ = robot_x + cos(robot_yaw) ;
  mark_y_ = robot_y + sin(robot_yaw) ;

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  ROS_INFO_STREAM("Update cost:" << min_i << "," << min_j << "," << max_i << "," << max_j);
  if (constr_points_.empty())
    return;

  tf::StampedTransform transform;
  try
  {
    tf_->lookupTransform(global_frame_, frame_id_, ros::Time(0), transform);
    ROS_INFO_STREAM("Transform from : " << frame_id_ << " to " << global_frame_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  
  for(auto const& point: constr_points_)
  {
    updateCellCost(master_grid, point, transform); 
  }
    
}
void SimpleLayer::updateCellCost(costmap_2d::Costmap2D& master_grid, tf::Point point, const tf::StampedTransform transform)
                                          
{
  unsigned int mx;
  unsigned int my;
  tf::Point p(point.x(), point.y(), 0);
  p = transform(p);
  
  if(master_grid.worldToMap(p.x(), p.y(), mx, my)){
    ROS_INFO_STREAM("Set cost for x:" << mx << " y:" << my); 
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}
void SimpleLayer::constrCb(const sparrow_planner::points& msg)
{
  
  for(auto const& point: msg.points)
  {
    constr_points_.push_back(tf::Point(point.x, point.y, 0));
  }
}
} // end namespace

