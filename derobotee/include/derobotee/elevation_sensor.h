#include "ros/ros.h"
#include "controlcan.h"
#include "derobotee/Elevation.h"
namespace derobotee {
class ElevationSensor
{

public:
        ElevationSensor();
	void spin();
        void update();
        void init_canbus();
private:
        VCI_CAN_OBJ rec[3000];
        unsigned int port;
        ros::NodeHandle n;
        ros::Publisher elevation_publisher;
        int count;
};
}
