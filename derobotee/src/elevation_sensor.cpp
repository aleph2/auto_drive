#include <derobotee/elevation_sensor.h>
using namespace derobotee;


ElevationSensor::ElevationSensor():count(0)
{
    VCI_BOARD_INFO pInfo;
    printf(">>this is hello !\r\n");//指示程序已运行
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
            printf(">>open deivce success!\n");//打开设备成功
    }else
    {
            printf(">>open deivce error!\n");
            exit(1);
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
        {
                printf(">>Get VCI_ReadBoardInfo success!\n");

                //printf(" %08X", pInfo.hw_Version);printf("\n");
                //printf(" %08X", pInfo.fw_Version);printf("\n");
                //printf(" %08X", pInfo.dr_Version);printf("\n");
                //printf(" %08X", pInfo.in_Version);printf("\n");
                //printf(" %08X", pInfo.irq_Num);printf("\n");
                //printf(" %08X", pInfo.can_Num);printf("\n");
                printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
                printf("%c", pInfo.str_Serial_Num[1]);
                printf("%c", pInfo.str_Serial_Num[2]);
                printf("%c", pInfo.str_Serial_Num[3]);
                printf("%c", pInfo.str_Serial_Num[4]);
                printf("%c", pInfo.str_Serial_Num[5]);
                printf("%c", pInfo.str_Serial_Num[6]);
                printf("%c", pInfo.str_Serial_Num[7]);
                printf("%c", pInfo.str_Serial_Num[8]);
                printf("%c", pInfo.str_Serial_Num[9]);
                printf("%c", pInfo.str_Serial_Num[10]);
                printf("%c", pInfo.str_Serial_Num[11]);
                printf("%c", pInfo.str_Serial_Num[12]);
                printf("%c", pInfo.str_Serial_Num[13]);
                printf("%c", pInfo.str_Serial_Num[14]);
                printf("%c", pInfo.str_Serial_Num[15]);
                printf("%c", pInfo.str_Serial_Num[16]);
                printf("%c", pInfo.str_Serial_Num[17]);
                printf("%c", pInfo.str_Serial_Num[18]);
                printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

                printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
                printf("%c", pInfo.str_hw_Type[1]);
                printf("%c", pInfo.str_hw_Type[2]);
                printf("%c", pInfo.str_hw_Type[3]);
                printf("%c", pInfo.str_hw_Type[4]);
                printf("%c", pInfo.str_hw_Type[5]);
                printf("%c", pInfo.str_hw_Type[6]);
                printf("%c", pInfo.str_hw_Type[7]);
                printf("%c", pInfo.str_hw_Type[8]);
                printf("%c", pInfo.str_hw_Type[9]);printf("\n");
        }else
        {
                printf(">>Get VCI_ReadBoardInfo error!\n");
                exit(1);
        }

        VCI_INIT_CONFIG config;
        config.AccCode=0;
        config.AccMask=0xFFFFFFFF;
        config.Filter=1;//接收所有帧
        config.Timing0=0x01;/*波特率125 Kbps  0x03  0x1C*/
        config.Timing1=0x1C;
        config.Mode=0;//正常模式                

        if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
        {
                printf(">>Init CAN1 error\n");
                VCI_CloseDevice(VCI_USBCAN2,0);
        }

        if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
        {
                printf(">>Start CAN1 error\n");
                VCI_CloseDevice(VCI_USBCAN2,0);

        }
       VCI_ResetCAN(VCI_USBCAN2, 0, 0);

       elevation_publisher = n.advertise<derobotee::Elevation>("elevation", 50, 0);
}

void ElevationSensor::spin()
{
    ros::Rate loop_rate(10);

    while (ros::ok())
       {
               update();
               loop_rate.sleep();
       }


}
void ElevationSensor::update()
{
        int reclen=0;
        int i,j;
        int ind=0;
        unsigned char h[4];
        if((reclen=VCI_Receive(VCI_USBCAN2,0,0,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
        {
            derobotee::Elevation msg;
            for(j=0;j<reclen;j++)
            {
                 if(rec[j].ID != 0x000004C0 || rec[j].DataLen <= 1) {
                     continue;
                 }
/*
                 printf("Index:%04d  ",count);count++;//序号递增
                 printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
                 if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
                 if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
                 if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
                 if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
  */
                 for(i = 0; i < rec[j].DataLen; i++)
                 {
                     h[3-i] = rec[j].Data[i];
                 }
                 int *p = (int *)h;
                 msg.data = *p; 
                 elevation_publisher.publish(msg);
            }
        }
//        spin();
}
int main(int argc, char **argv)
{

        ros::init(argc, argv,"elevation_sensor");
        ElevationSensor ds;
        ds.spin();
        return 0;

}
