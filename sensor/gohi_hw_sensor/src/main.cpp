#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gohi_hw_sensor");
    ros::NodeHandle nh("gohi_hw");

    HIGO_ROS higo(nh, "tcp", "/home/zhuxi/gohi_ws/src/GOHI_ROBOT/sensor/gohi_hw_sensor/config.txt");

    higo.mainloop();
    return 0;
}
