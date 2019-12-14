#include <libserial/SerialPort.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>

constexpr const char* const SERIAL_PORT_1 = "/dev/IMU" ;

using namespace LibSerial ;

class ImuDriver{

public:
  ImuDriver();
  void init();
  
//  void spin();
private:
//  ros::NodeHandle n;
  SerialPort serial_port_1;
};

ImuDriver::ImuDriver()
{
}

void ImuDriver::init()
{
}

int main(int argc, char** argv)
{
  printf("hello\n\r");
  return 0;
}

