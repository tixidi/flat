#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw_dtu");
    ros::NodeHandle nh("gohi_hw_dtu");

    // HIGO_ROS higo(nh, "tcp", "/home/wb/gohi_ws/src/GOHI_ROBOT/dtu/gohi_hw_monitor/config.txt");
    // HIGO_ROS higo(nh, "udp", "/home/wb/gohi_ws/src/GOHI_ROBOT/dtu/gohi_hw_monitor/config.txt");
    HIGO_ROS higo(nh, "tcp_server", "/home/yinhe/gohi_ws/src/GOHI_ROBOT/dru/gohi_hw_monitor/config.txt");
    
    

    higo.mainloop();
    return 0;
}
