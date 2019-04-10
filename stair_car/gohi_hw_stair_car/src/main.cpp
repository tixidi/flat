#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stair_car_hw");
    ros::NodeHandle nh("stair_car");

    HIGO_ROS higo(nh, "tcp", "/home/zhuxi/gohi_ws/src/GOHI_ROBOT/stair_car/gohi_hw_stair_car/config.txt");

    higo.mainloop();
    return 0;
}
