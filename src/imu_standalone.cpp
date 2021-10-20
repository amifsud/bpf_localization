#define LOG_LEVEL Info

#include "bpf_localization/sensors.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_standalone");
    ros::NodeHandle nh;

    auto imu = IMUInterface(&nh, "boat_imu", 3);

    ros::Rate r(5.);

    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
