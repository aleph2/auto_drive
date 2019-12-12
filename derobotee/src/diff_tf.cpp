#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <inttypes.h>
#include "derobotee/MotorStatusList.h"

class Odometry_calc{

public:
	Odometry_calc();

	void spin();


private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber l_wheel_sub;
	ros::Subscriber r_wheel_sub;
	ros::Subscriber wheels_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_lencoder;
	double prev_rencoder;

	double lmult;
	double rmult;

	double left;
	double right;

	double rate;

	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;

	int left_idx;
        int right_idx;
        int l_direction;
        int r_direction;

	double enc_left ;

	double enc_right;

	double ticks_meter;

	double base_width;

	double dx;

	double dr;

	double x_final,y_final, theta_final;

	ros::Time current_time, last_time;


	void leftencoderCb(int32_t left_ticks );
	void rightencoderCb(int32_t left_ticks );
/*
	void leftencoderCb(const std_msgs::Int64::ConstPtr& left_ticks);
	void rightencoderCb(const std_msgs::Int64::ConstPtr& right_ticks);
*/
	void encoderCb(const derobotee::MotorStatusList::ConstPtr& motor_states);

	void init_variables();

	void get_node_params();


	void update();
};

Odometry_calc::Odometry_calc(){


	init_variables();

	ROS_INFO("Started odometry computing node");
/*
	l_wheel_sub = n.subscribe("/lwheel",10, &Odometry_calc::leftencoderCb, this);
	
	r_wheel_sub = n.subscribe("/rwheel",10, &Odometry_calc::rightencoderCb, this);
*/
	wheels_sub = n.subscribe("/motor/state",10, &Odometry_calc::encoderCb, this);


  	odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);   
  	


	//Retrieving parameters of this node
	get_node_params();
}

void Odometry_calc::init_variables()
{


	prev_lencoder = 0;
	prev_rencoder = 0;

	lmult = 0;
	rmult = 0;

	left = 0;
	right = 0;

	encoder_min =  -2147483648;
	encoder_max =  2147483648;

	rate = 10;

	//ticks_meter = 50;

	base_width = 0.3;

	
	

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;
	
	then = ros::Time::now();


	enc_left = 0;
	enc_right = 0;

	dx = 0;
	dr = 0;
 
	x_final = 0;y_final=0;theta_final=0;
	
	current_time = ros::Time::now();
  	last_time = ros::Time::now();

}
void Odometry_calc::get_node_params(){

	
        if(n.getParam("rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}

        if(n.getParam("encoder_min", encoder_min)){
	 
		ROS_INFO_STREAM("Encoder min from param" << encoder_min);	       
	}


        if(n.getParam("encoder_max", encoder_max)){
	 
		ROS_INFO_STREAM("Encoder max from param" << encoder_max);	       
	}


        if(n.getParam("wheel_low_wrap", encoder_low_wrap)){
	 
		ROS_INFO_STREAM("wheel_low_wrap from param" << encoder_low_wrap);	       
	}


        if(n.getParam("wheel_high_wrap", encoder_high_wrap)){
	 
		ROS_INFO_STREAM("wheel_high_wrap from param" << encoder_high_wrap);	       
	}




        if(n.getParam("/derobotee/ticks_meter", ticks_meter)){
	 
		ROS_INFO_STREAM("Ticks meter" << ticks_meter);	       
	}


        if(n.getParam("/derobotee/base_width", base_width )){
	 
		ROS_INFO_STREAM("Base Width" << base_width );	       
	}

        if(n.getParam("/motor/left_wheel", left_idx)){

                ROS_INFO_STREAM("Left wheel idx from param" << left_idx);
        }

        if(n.getParam("/motor/right_wheel", right_idx)){

                ROS_INFO_STREAM("Right wheel idx from param" << right_idx);
        }

        std::ostringstream key_left_stream;
        key_left_stream << "/motor/direction/" << left_idx;
        if(n.getParam(key_left_stream.str(), l_direction)){

                ROS_INFO_STREAM("Left direction from param" << l_direction);
        }

        std::ostringstream key_right_stream;
        key_right_stream << "/motor/direction/" << right_idx;
        if(n.getParam(key_right_stream.str(), r_direction)){

                ROS_INFO_STREAM("Right direction from param" << r_direction);
        }

/*
	ROS_INFO_STREAM("Encoder min" << encoder_min);
	ROS_INFO_STREAM("Encoder max" << encoder_max);

	ROS_INFO_STREAM("Encoder Low Wrap"<< encoder_low_wrap);
	ROS_INFO_STREAM("Encoder High Wrap" << encoder_high_wrap);

	ROS_INFO_STREAM("ticks meter" << ticks_meter);
	ROS_INFO_STREAM("base width" << base_width);

*/

}
//Spin function
void Odometry_calc::spin(){

     ros::Rate loop_rate(rate);

     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	
	}


}

//Update function
void Odometry_calc::update(){

	ros::Time now = ros::Time::now();
	
//	ros::Time elapsed;

	double elapsed;

	double d_left, d_right, d, th,x,y;


	if ( now > t_next) {

		elapsed = now.toSec() - then.toSec(); 

 // 	        ROS_INFO_STREAM("elapsed =" << elapsed);

		
		
		if(enc_left == 0){
			d_left = 0;
			d_right = 0;
		}
		else{
			d_left = (left - enc_left) / ( ticks_meter);
			d_right = (right - enc_right) / ( ticks_meter);
		}
		enc_left = left;
		enc_right = right;

		d = (d_left + d_right ) / 2.0;

//		ROS_INFO_STREAM("d" << d);


		th = ( d_right - d_left ) / base_width;
		
		dx = d /elapsed;

		dr = th / elapsed;

	
		if ( d != 0){

                	x = cos( th ) * d;
                	y = sin( th ) * d;
                	// calculate the final position of the robot
                	x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
                	y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );

			}

           	 if( th != 0)
                	theta_final = theta_final + th;

		    geometry_msgs::Quaternion odom_quat ;

		    odom_quat.x = 0.0;
		    odom_quat.y = 0.0;
		    odom_quat.z = 0.0;

            	    odom_quat.z = sin( theta_final / 2 );	
            	    odom_quat.w = cos( theta_final / 2 );

		    //first, we'll publish the transform over tf
		    geometry_msgs::TransformStamped odom_trans;
		    odom_trans.header.stamp = now;
		    odom_trans.header.frame_id = "odom";
		    odom_trans.child_frame_id = "base_footprint";

		    odom_trans.transform.translation.x = x_final;
		    odom_trans.transform.translation.y = y_final;
		    odom_trans.transform.translation.z = 0.0;
		    odom_trans.transform.rotation = odom_quat;

		    //send the transform
		    odom_broadcaster.sendTransform(odom_trans);

		    
		    //next, we'll publish the odometry message over ROS
		    nav_msgs::Odometry odom;
		    odom.header.stamp = now;
		    odom.header.frame_id = "odom";

		    //set the position
		    odom.pose.pose.position.x = x_final;
		    odom.pose.pose.position.y = y_final;
		    odom.pose.pose.position.z = 0.0;
		    odom.pose.pose.orientation = odom_quat;

		    //set the velocity
		    odom.child_frame_id = "base_footprint";
		    odom.twist.twist.linear.x = dx;
		    odom.twist.twist.linear.y = 0;
		    odom.twist.twist.angular.z = dr;

		    //publish the message
		    odom_pub.publish(odom);

	    	    then = now;

//		    ROS_INFO_STREAM("dx =" << x_final);

//		    ROS_INFO_STREAM("dy =" << y_final);

	            ros::spinOnce();


		}
	 else { ; }
//		ROS_INFO_STREAM("Not in loop");
		
		



}


void Odometry_calc::encoderCb(const derobotee::MotorStatusList::ConstPtr& motor_states)
{
    //ROS_INFO_STREAM("receive state as " );
    for(int i = 0; i < motor_states->list.size(); i++)
    {
        if(motor_states->list[i].slave == left_idx)
	{
	    leftencoderCb(l_direction * motor_states->list[i].actualPosition);
	    continue;
	}
        if(motor_states->list[i].slave == right_idx)
	{
	    rightencoderCb(r_direction * motor_states->list[i].actualPosition);
	    continue;
	}
    }
     
}


void Odometry_calc::leftencoderCb(int32_t left_ticks)

{

// ROS_INFO_STREAM("Left tick" << left_ticks->data);
	double enc = left_ticks;

	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		
		lmult = lmult + 1;
	}
	

	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))

	{
		
		lmult = lmult - 1;
	}

	left = 1.0 * (enc + lmult * (encoder_max - encoder_min ));

	prev_lencoder = enc;

//	ROS_INFO_STREAM("Left " << left);

}


//Right encoder callback

void Odometry_calc::rightencoderCb(int32_t right_ticks)

{


// ROS_INFO_STREAM("Right tick" << right_ticks->data);


	double enc = right_ticks;

	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		
		rmult = rmult + 1;
	}
	

	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))

	{
		
		rmult = rmult - 1;
	}

	right = 1.0 * (enc + rmult * (encoder_max - encoder_min ));

	prev_rencoder = enc;

//	ROS_INFO_STREAM("Right " << right);



}




int main(int argc, char **argv)

{
	ros::init(argc, argv,"diff_tf");
	Odometry_calc obj;
	obj.spin();


	return 0;

}
