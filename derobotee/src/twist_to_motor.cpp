#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include "derobotee/MotorCmdList.h"
#define v2rpm(v, wheel_radius) (wheel_gear_ratio* v*30/(M_PI*wheel_radius))
class TwistToMotors
{

public:
	TwistToMotors();
	void spin();

private:
	ros::NodeHandle n;
	
	ros::Publisher pub_lmotor;
	ros::Publisher pub_rmotor;
	ros::Publisher pub_motor;

	ros::Subscriber cmd_vel_sub;
        int left_idx;
        int right_idx;

	float left;
	float right;

	float ticks_since_target;
	double timeout_ticks;

	double w;
	double rate;
	double left_wheel_radius;
	double right_wheel_radius;
	double wheel_gear_ratio;

	float dx,dy,dr;

        double z_compensate;
	void init_variables();
	void get_parameters();

	void spinOnce();
	void twistCallback(const geometry_msgs::Twist &twist_aux);

};

TwistToMotors::TwistToMotors()
{
	init_variables();
	get_parameters();
	
	ROS_INFO("Started Twist to Motor node");
	
	cmd_vel_sub = n.subscribe("/twist_mux/cmd_vel",10, &TwistToMotors::twistCallback, this);
	
	pub_lmotor = n.advertise<std_msgs::Float32>("lwheel_vtarget", 50);

	pub_rmotor = n.advertise<std_msgs::Float32>("rwheel_vtarget", 50);

	pub_motor = n.advertise<derobotee::MotorCmdList>("cmd", 50);
	


}

void TwistToMotors::init_variables()
{
	left = 0;
	right = 0;

	dx = dy = dr =0;

	w = 0.2 ;
	rate = 50;
	timeout_ticks = 2;



}


void TwistToMotors::get_parameters()
{


	
        if(n.getParam("rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}


	
        if(n.getParam("timeout_ticks", timeout_ticks)){
	 
		ROS_INFO_STREAM("timeout_ticks from param" << timeout_ticks);	       
	}

	
        if(n.getParam("base_width", w)){
	 
		ROS_INFO_STREAM("Base_width from param" << w);	       
	}

        n.param<double>("z_compensate", z_compensate, 0.01);
        n.param<double>("left_wheel_radius", left_wheel_radius, 15.0);
        n.param<double>("right_wheel_radius", right_wheel_radius, 15.0);
        ROS_INFO_STREAM("wheel raidus left righ is " << left_wheel_radius << " " << right_wheel_radius);
        if(n.getParam("wheel_gear_ratio", wheel_gear_ratio)){
	 
		ROS_INFO_STREAM("wheel radius from param" << wheel_gear_ratio);	       
	}

        if(n.getParam("/motor/left_wheel", left_idx)){
	 
		ROS_INFO_STREAM("Left wheel idx from param" << left_idx);	       
	}

        if(n.getParam("/motor/right_wheel", right_idx)){
	 
		ROS_INFO_STREAM("Right wheel idx from param" << right_idx);	       
	}

}


void TwistToMotors::spin()
{
	ros::Rate r(rate);
	ros::Rate idle(10);

	ros::Time then = ros::Time::now();
	
	ticks_since_target = timeout_ticks;

	

	while (ros::ok())
	{
	while (ros::ok() && (ticks_since_target <= timeout_ticks))	
		{		

		spinOnce();
		r.sleep();

		}
	ros::spinOnce();
        idle.sleep();	

	}

}

void TwistToMotors::spinOnce()
{


        // dx = (l + r) / 2
        // dr = (r - l) / w

	
//	right = ( 1.0 * dx ) + (dr * w /2);
//	left = ( 1.0 * dx ) - (dr * w /2);
	right = dx  + (dr * w /2);
	left = dx  - (dr * w /2);


//	ROS_INFO_STREAM("right = " << right << "\t" << "left = " << left << "dr"<< dr);


	std_msgs::Float32 left_;
	std_msgs::Float32 right_;

	left_.data = left;
	right_.data = right;

        derobotee::MotorCmdList cmds;
        derobotee::MotorCmd cmdL;
        derobotee::MotorCmd cmdR;
        cmdL.slave = left_idx;
        cmdL.value = v2rpm(left, left_wheel_radius);
        cmdR.slave = right_idx;
        cmdR.value = v2rpm(right, right_wheel_radius);
        cmds.list.push_back(cmdL);
        cmds.list.push_back(cmdR);
   
	pub_lmotor.publish(left_);
	pub_rmotor.publish(right_);
	pub_motor.publish(cmds);

	ticks_since_target += 1;

	ros::spinOnce();


}

void TwistToMotors::twistCallback(const geometry_msgs::Twist &msg)
{

	ticks_since_target = 0;
	
	dx = msg.linear.x;
	dy = msg.linear.y;
	dr = msg.angular.z;

}


int main(int argc, char **argv)
{

	ros::init(argc, argv,"twist_to_motor");
	TwistToMotors obj;

	obj.spin();


}
