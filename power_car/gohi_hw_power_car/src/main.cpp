#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "power_car_hw");
    ros::NodeHandle nh("power_car");

    HIGO_ROS higo(nh, "tcp", "/home/zhuxi/gohi_ws/src/GOHI_ROBOT/power_car/gohi_hw_power_car/config.txt");

    higo.mainloop();
    return 0;
}
