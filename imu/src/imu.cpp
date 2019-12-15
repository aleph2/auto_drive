#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include <arpa/inet.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

#include <serial/serial.h>

typedef struct
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t theta_x;
  int16_t theta_y;
  int16_t theta_z;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t temp;
  int8_t check_sum;
  uint16_t teminator;
} imu_data;
static const double gravity_factor_ = 9.80665;
static const double linear_fact_ = gravity_factor_*20/65536.0;
static const double theta_fact_ = M_PI*7/65536.0;
static const double pose_fact_ = M_PI*2/65536.0;

class ImuDriver{

public:
  ImuDriver();
  int init();
  void spin();  
//  void spin();
private:
  ros::NodeHandle n;
  serial::Serial* my_serial_;
  ros::Publisher imu_pub_;
  void update();
};

void ImuDriver::spin()
{
  ros::Rate loop_rate(200);
  while (ros::ok())
  {
    update();
    loop_rate.sleep();
  }
}
ImuDriver::ImuDriver()
{
}
void ImuDriver::update()
{
  
}
int ImuDriver::init()
{
  std::string port("/dev/IMU");
  uint32_t  baud = 115200;
  imu_pub_ = n.advertise<sensor_msgs::Imu>("/odom", 50);
  my_serial_ = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
  if(my_serial_->isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;

  while(true)
  {
    std::string dataString;
    imu_data* p;
    my_serial_->readline(dataString, 1024, "\x7f\x80");
    if(dataString.size() < 23)
    {
       continue;
    }
    p = (imu_data*)dataString.c_str();
    tf2::Quaternion q;
    q.setRPY(p->roll*pose_fact_, p->pitch*pose_fact_, p->yaw*pose_fact_);
    sensor_msgs::Imu msg;
    msg.linear_acceleration.x = (p->acc_x)*linear_fact_ ;
    msg.linear_acceleration.y = (p->acc_y)*linear_fact_ ;
    msg.linear_acceleration.z = (p->acc_z)*linear_fact_ ;
    msg.angular_velocity.x = (p->theta_x)*theta_fact_ ;
    msg.angular_velocity.y = (p->theta_y)*theta_fact_ ;
    msg.angular_velocity.z = (p->theta_z)*theta_fact_ ;
    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    imu_pub_.publish(msg); 
  }
}

int main(int argc, char** argv)
{
 ros::init(argc, argv,"imu_node");
 if ( htonl(47) == 47 ) {
  // Big endian
  printf("Big Endian");
}
  ImuDriver imu;
  imu.init();
  return 0;
}

