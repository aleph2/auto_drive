#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <inttypes.h>
#include <vector>
extern "C" {
#include <derobotee/ethercat.h>
/*
#include <derobotee/ethercattype.h>
#include <derobotee/nicdrv.h>
#include <derobotee/ethercatbase.h>
#include <derobotee/ethercatmain.h>
#include <derobotee/ethercatdc.h>
#include <derobotee/ethercatcoe.h>
#include <derobotee/ethercatfoe.h>
#include <derobotee/ethercatsoe.h>
#include <derobotee/ethercatconfig.h>
#include <derobotee/ethercatprint.h>
*/
}

#include "derobotee/MotorCmd.h"
#include "derobotee/MotorStatus.h"
#include "derobotee/MotorCmdList.h"
#include "derobotee/MotorStatusList.h"

#define EC_TIMEOUTMON 500
typedef struct PACKED {
        uint16 ctrlWd;
        int32 targetVelocity;
        uint8 opMode;
} out_frame;

typedef struct  PACKED {
        uint16 statusWord;
        int32 actualPosition;
        int32 actualVelocity;
        uint8 opMode;
} in_frame;

namespace derobotee {
class DrSoem
{

public:
        DrSoem();
        void spin();
        void ecatcheck(const ros::TimerEvent&);

private:
        char IOmap[4096];
        int expectedWKC;
        boolean needlf;
        volatile int wkc;
        boolean inOP;
        uint8 currentgroup;
        ros::NodeHandle n;

        ros::Publisher motorsState;

        ros::Subscriber cmds2MotorsSub;

        double rate;
        std::string ifname;
        void init_variables();
        void get_parameters();
        void init_ethercat();
        void initSpeedMode(unsigned int slave);
        void initPositionMode(unsigned int slave);

        void spinOnce();//at the end of main loop
        void processCmds(const derobotee::MotorCmdList::ConstPtr& msg);
        void update();
        void initEthercat();
        void transferData();
        void configSpeedPDO(int idx);
        void configPositionPDO(int idx);
        void releaseMotor(unsigned int s);
//variable for ec data process
        int i, j, oloop, iloop, chk;
        void configSpeedMode(unsigned int slave);
        void configPositionMode(unsigned int slave);
        void initSpeedTPDO(unsigned int slave);
        void initPositionTPDO(unsigned int slave);
        in_frame*  m_inputs;
        out_frame* m_outputs;
        int total_motor;
        std::vector<int> directions;
        std::vector<std::string> modes;
        int max_position;
        int min_position;
        int grinder_idx;
        bool motors_initialized_;
};

}
