#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw_dtu");
    ros::NodeHandle nh("gohi_hw_dtu");

    // HIGO_ROS higo(nh, "tcp", "/home/wb/gohi_ws/src/GOHI_ROBOT/dtu/gohi_hw_rev/config.txt");
    HIGO_ROS higo(nh, "udp", "/home/yinhe/gohi_ws/src/GOHI_ROBOT/dtu/gohi_hw_rev/config.txt");
    

    higo.mainloop();
    return 0;
}
