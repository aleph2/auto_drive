#include <derobotee/dr_soem.h>
/**
subscribe msg as motorNumber:cmdType:value
publish msg as motorNumber:position:status:timestamp
*/
using namespace derobotee;
DrSoem::DrSoem():directions(10, 0), modes(10,""), min_position(-1),max_position(200000)
{
	init_variables();
	get_parameters();
        initEthercat();	
        for(int i = 1; i <= ec_slavecount; i++)
        {
            if(modes[i].compare("speed") == 0){
                configSpeedPDO(i);
                initSpeedTPDO(i);
                initSpeedMode(i);

            } else {
                configPositionPDO(i);
                initPositionTPDO(i);
                initPositionMode(i);
            }
        }
	ROS_INFO("Started ethercat driver node");
	
	cmds2MotorsSub  = n.subscribe("cmd",10, &DrSoem::processCmds, this);

	ROS_INFO("Subscribe MotorCmdsList");
	
	motorsState = n.advertise<derobotee::MotorStatusList>("state", 50);
	ROS_INFO("Publish MotorStatusList");
//        ros::Timer timer = n.createTimer(ros::Duration(0.1), &DrSoem::ecatcheck, this);

}

void DrSoem::init_variables()
{

	rate = 50;
	currentgroup = 0;
}


void DrSoem::get_parameters()
{
	
        if(n.getParam("/motor/rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}

        if(n.getParam("/motor/ifname", ifname)){
	 
		ROS_INFO("ifname from param %s", ifname.c_str());	       
	}else 
        {
                ROS_INFO("didn't found ifname");
        }
        if(n.getParam("/derobotee/total_motor", total_motor)){
	 
		ROS_INFO_STREAM("We have " << total_motor << " motors");	       
	}
        if(n.getParam("/derobotee/min_position", min_position)){
	 
		ROS_INFO_STREAM("min position value is " << min_position );
	}
        if(n.getParam("/derobotee/max_position", max_position)){
	 
		ROS_INFO_STREAM("max position value is " << max_position);
	}
        int d;
        for(int i = 1; i <= total_motor; i++) {
           std::ostringstream key_stream;
           key_stream << "/motor/direction/" << i;
           if(n.getParam(key_stream.str(), d)){
             ROS_INFO_STREAM("The " << i << "'th motor's direction is " << d);
             directions[i]=d; 
           }else {
             ROS_INFO_STREAM("Param of " << key_stream.str() << " was not found");
            
           }
        }
        std::string mode;
        for(int i = 1; i <= total_motor; i++) {
           std::ostringstream key_stream;
           key_stream << "/motor/mode/" << i;
           if(n.getParam(key_stream.str(), mode)){
             ROS_INFO_STREAM("The " << i << "'th motor's mode is " << mode);
             modes[i]=mode; 
           }
        }
        n.param<int>("/motor/grinder", grinder_idx, 4);

}


void DrSoem::spin()
{
    ros::Rate loop_rate(rate);

    while (ros::ok())
       {
               update();
               loop_rate.sleep();
       }

}

void DrSoem::update()
{
    //receive process data
    transferData(); 
    ros::spinOnce();
}

void DrSoem::processCmds(const derobotee::MotorCmdList::ConstPtr& msg)
{
//TODO:
//proess recevied cmd
/*
    ROS_INFO("Recieve subscribed slave number: %d", msg->slave);
    ROS_INFO("Recieve value setting for slave %d ", msg->value);
    int vel = msg->value;
*/
//    ec_SDOwrite(msg->slave, 0x60FF, 0, FALSE, sizeof(int32),&vel, EC_TIMEOUTRXM);
//value is rpm, convert it to data sent to motor
    for(int i = 0; i < msg->list.size(); i++)
    {
        int s_idx = msg->list[i].slave;

        if(modes[s_idx].compare("speed") == 0){
            m_outputs = ((out_frame*)ec_slave[s_idx].outputs);
            m_outputs->targetVelocity = (int)msg->list[i].value * 2730 * directions[s_idx];
            m_outputs->ctrlWd = 0x0F;
            m_outputs->opMode = 3;
            if(s_idx == grinder_idx){
               m_outputs->opMode = -3;
            }
        }
        if(modes[s_idx].compare("position") == 0){
            m_outputs = ((out_frame*)ec_slave[s_idx].outputs);
            m_outputs->targetVelocity = std::max(min_position, std::min(max_position,(int)msg->list[i].value));
            m_outputs->ctrlWd = 0x3F;
            m_outputs->opMode = 1;
	}
    }
}

void DrSoem::initEthercat()
{
    if (ec_init(const_cast<char*>(ifname.c_str())))
   {
      ROS_INFO_STREAM("ec_init on " << ifname << " successed");
      /* find and auto-config slaves */


       if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         ec_configdc();
         ec_dcsync0(0, TRUE, 4000*1000, 10000);

         ec_config_map(&IOmap);

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 40;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
       }
    } else {
      ROS_INFO("init network failed!");
    }
}

void DrSoem::transferData()
{
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if(wkc >=expectedWKC)
    {
         derobotee::MotorStatusList msgs;
         for(int i = 1; i <= ec_slavecount; i++)
         {
             m_inputs = ((in_frame*)(ec_slave[i].inputs)); 
             derobotee::MotorStatus msg;
             msg.statusWord = m_inputs->statusWord;
             msg.actualPosition = m_inputs->actualPosition;
/*
             if(m_inputs->actualVelocity != 0)
             {
                 ROS_INFO_STREAM("The actual vel of the slave #" <<i <<" is not zero:" << m_inputs->actualVelocity);
             }else {
                 ROS_INFO("The actual vel is zero" );
             }
*/  
           msg.actualVelocity = m_inputs->actualVelocity;
             msg.opMode = m_inputs->opMode;
             msg.slave = i;
             msgs.list.push_back(msg);
         } 
         
         motorsState.publish(msgs);
         
/*         printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

         for(j = 0 ; j < oloop; j++)
         {
             printf(" %2.2x", *(ec_slave[0].outputs + j));
         }

         printf(" I:");
         for(j = 0 ; j < iloop; j++)
         {
              printf(" %2.2x", *(ec_slave[0].inputs + j));
         }
         printf(" T:%"PRId64"\r",ec_DCtime);
                  //  needlf = TRUE;
*/
     }
  
     osal_usleep(4000);

   
}

void DrSoem::initPositionMode(unsigned int s)
{
   int8 positionMode = 1;
   uint16 ctrlReset = 0x86;
   uint16 ctrlLock = 0x02F;
   uint16 ctrlUnlock = 0x03F;
   uint32 acc = 100*2757;
   uint32 dacc = 100*2757;
   int32 velocity = 100*2757;//-100*2757;
   ec_SDOwrite(s, 0x6040, 0, FALSE, sizeof(uint16),&ctrlReset, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6040, 0, FALSE, sizeof(uint16),&ctrlUnlock, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6040, 0, FALSE, sizeof(uint16),&ctrlLock, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6081, 0, FALSE, sizeof(uint32),&acc, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6083, 0, FALSE, sizeof(uint32),&dacc, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6084, 0, FALSE, sizeof(uint32),&acc, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x60FF, 0, FALSE, sizeof(int32),&velocity, EC_TIMEOUTRXM);
}

void DrSoem::initSpeedMode(unsigned int s)
{
   int8 speedMode = -3;
   uint16 ctrlReset = 0x86;
   uint16 ctrlLock = 0x0F;
   uint16 ctrlUnlock = 0x06;

   uint32 acc = 20*2757;
   uint32 dacc = 20*2757;
   uint32 t_v = 100*2757;
   if(s == grinder_idx)
   {
     acc = 3*2757;
     dacc = 3*2757;
   }
   int32 speed = 0;//-100*2757;
   ec_SDOwrite(s, 0x6040, 0, FALSE, sizeof(uint16),&ctrlReset, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6040, 0, FALSE, sizeof(uint16),&ctrlUnlock, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6040, 0, FALSE, sizeof(uint16),&ctrlLock, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6081, 0, FALSE, sizeof(uint32),&t_v, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6083, 0, FALSE, sizeof(uint32),&acc, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x6084, 0, FALSE, sizeof(uint32),&dacc, EC_TIMEOUTRXM);
   ec_SDOwrite(s, 0x60FF, 0, FALSE, sizeof(int32),&speed, EC_TIMEOUTRXM);
}

void DrSoem::initPositionTPDO(unsigned int s)
{
    m_outputs = ((out_frame*)ec_slave[s].outputs);
    m_outputs->targetVelocity = 0;
    m_outputs->opMode = 1;
    m_outputs->ctrlWd = 0x03F;
}
void DrSoem::initSpeedTPDO(unsigned int s)
{
    m_outputs = ((out_frame*)ec_slave[s].outputs);
    m_outputs->targetVelocity = 0;
    m_outputs->opMode = -3;
    m_outputs->ctrlWd = 0x0F;
}
void DrSoem::configPositionPDO(int s)
{

    uint8 disable = 0x0;
    uint8 enable = 0x1;
    uint32 ctrlWd = 0x60400010;
    uint32 targetP = 0x607a0020;
//    uint32 targetP = 0x607a0020;
//    uint32 profileAcc = 0x60830020;
//    uint32 profileDacc = 0x60840020;
    uint32 opMode = 0x60600008;
    uint16 idx = 0x1600;
    uint8 vcount = 0x3;
    ec_SDOwrite(s, 0x1c12, 0, FALSE, sizeof(uint8), &disable, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x1, FALSE, sizeof(uint32), &ctrlWd, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x2, FALSE, sizeof(uint32), &targetP, EC_TIMEOUTRXM);
//    ec_SDOwrite(1, 0x1600, 0x2, FALSE, sizeof(uint32), &targetP, EC_TIMEOUTRXM);
//    ec_SDOwrite(1, 0x1600, 0x3, FALSE, sizeof(uint32), &profileAcc, EC_TIMEOUTRXM);
//    ec_SDOwrite(1, 0x1600, 0x4, FALSE, sizeof(uint32), &profileDacc, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x3, FALSE, sizeof(uint32), &opMode, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x0, FALSE, sizeof(uint8), &vcount, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c12, 0x1, FALSE, sizeof(uint16), &idx, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c12, 0, FALSE, sizeof(uint8), &enable, EC_TIMEOUTRXM);

    uint32 statusWd = 0x60410010;
    uint32 actualPosition = 0x60630020;
    uint32 actualVel = 0x606c0020;
    opMode = 0x60610008;
    uint16 idx2 = 0x1a00;
    vcount = 4;
    ec_SDOwrite(s, 0x1c13, 0, FALSE, sizeof(uint8), &disable, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c13, 0x1, FALSE, sizeof(uint16), &idx2, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0, FALSE, sizeof(uint8), &disable, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x1, FALSE, sizeof(uint32), &statusWd, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x2, FALSE, sizeof(uint32), &actualPosition, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x3, FALSE, sizeof(uint32), &actualVel, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x4, FALSE, sizeof(uint32), &opMode, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x0, FALSE, sizeof(uint8), &vcount, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c13, 0, FALSE, sizeof(uint8), &enable, EC_TIMEOUTRXM);
    //setup mode by sdo write
    uint8 ctlMode = 1;
    ec_SDOwrite(s, 0x6060, 0, FALSE, sizeof(uint8), &ctlMode, EC_TIMEOUTRXM);


}
void DrSoem::configSpeedPDO(int s)
{

    uint8 disable = 0x0;
    uint8 enable = 0x1;
    uint32 ctrlWd = 0x60400010;
    uint32 targetV = 0x60ff0020;
//    uint32 targetP = 0x607a0020;
//    uint32 profileAcc = 0x60830020;
//    uint32 profileDacc = 0x60840020;
    uint32 opMode = 0x60600008;
    uint16 idx = 0x1600;
    uint8 vcount = 0x3;
    ec_SDOwrite(s, 0x1c12, 0, FALSE, sizeof(uint8), &disable, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x1, FALSE, sizeof(uint32), &ctrlWd, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x2, FALSE, sizeof(uint32), &targetV, EC_TIMEOUTRXM);
//    ec_SDOwrite(1, 0x1600, 0x2, FALSE, sizeof(uint32), &targetP, EC_TIMEOUTRXM);
//    ec_SDOwrite(1, 0x1600, 0x3, FALSE, sizeof(uint32), &profileAcc, EC_TIMEOUTRXM);
//    ec_SDOwrite(1, 0x1600, 0x4, FALSE, sizeof(uint32), &profileDacc, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x3, FALSE, sizeof(uint32), &opMode, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1600, 0x0, FALSE, sizeof(uint8), &vcount, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c12, 0x1, FALSE, sizeof(uint16), &idx, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c12, 0, FALSE, sizeof(uint8), &enable, EC_TIMEOUTRXM);

    uint32 statusWd = 0x60410010;
    uint32 actualPosition = 0x60630020;
    uint32 actualVel = 0x606c0020;
    opMode = 0x60610008;
    uint16 idx2 = 0x1a00;
    vcount = 4;
    ec_SDOwrite(s, 0x1c13, 0, FALSE, sizeof(uint8), &disable, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c13, 0x1, FALSE, sizeof(uint16), &idx2, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0, FALSE, sizeof(uint8), &disable, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x1, FALSE, sizeof(uint32), &statusWd, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x2, FALSE, sizeof(uint32), &actualPosition, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x3, FALSE, sizeof(uint32), &actualVel, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x4, FALSE, sizeof(uint32), &opMode, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1a00, 0x0, FALSE, sizeof(uint8), &vcount, EC_TIMEOUTRXM);
    ec_SDOwrite(s, 0x1c13, 0, FALSE, sizeof(uint8), &enable, EC_TIMEOUTRXM);
    //setup mode by sdo write
    uint8 ctlMode = 3;
    ec_SDOwrite(s, 0x6060, 0, FALSE, sizeof(uint8), &ctlMode, EC_TIMEOUTRXM);


}
void DrSoem::ecatcheck(const ros::TimerEvent&)

{
        ROS_INFO("timer callback for ethercat check");
        int slave;
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {   
            if (needlf)
            {   
               needlf = FALSE;
               printf("\n");
            }   
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {   
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {   
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {   
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }   
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {   
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }   
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {   
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {   
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }   
                  }   
                  else if(!ec_slave[slave].islost)
                  {   
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {   
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }   
                  }   
               }   
              if (ec_slave[slave].islost)
               {   
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }

}

int main(int argc, char **argv)
{

	ros::init(argc, argv,"soem_motor_driver");
	DrSoem obj;

	obj.spin();


}
