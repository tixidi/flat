#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gohi_hw_bms");
    ros::NodeHandle nh("gohi_hw_bms");

    HIGO_ROS higo(nh, "tcp", "/home/zhuxi/gohi_ws/src/GOHI_ROBOT/bms_battery/gohi_hw_bms/config.txt");

    higo.mainloop();
    return 0;
}
