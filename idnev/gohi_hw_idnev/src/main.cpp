#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gohi_hw_idnev");
    ros::NodeHandle nh("gohi_hw_idnev");

    HIGO_ROS higo(nh, "serial:///dev/ttyUSB1", "/home/zhuxi/gohi_ws/src/GOHI_ROBOT/idnev/gohi_hw_idnev/config.txt", \
                "/home/zhuxi/gohi_ws/src/GOHI_ROBOT/idnev/gohi_hw_idnev/idConfig.txt");

    higo.mainloop();
    return 0;
}
