#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gohi_hw_relay");
    ros::NodeHandle nh("gohi_hw_relay");

    HIGO_ROS higo(nh, "tcp", "/home/zhuxi/gohi_ws/src/GOHI_ROBOT/way_relay/gohi_hw_relay/config.txt");

    higo.mainloop();
    return 0;
}
