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
        std::deque<IntervalVector> data = imu.getIntervalData();
        for(auto i = 0; i < data.size(); ++i)
        {
            ROS_INFO_STREAM(data[i]);
        }
        ros::spinOnce();
        r.sleep();
    }
}
