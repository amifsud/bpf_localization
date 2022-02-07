#define LOG_LEVEL Info

#include "bpf_localization/interfaces/imu.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_standalone");
    ros::NodeHandle nh;

    std::string path = ros::package::getPath("bpf_localization");
    path += "/data/calibrations/";

    auto imu = Interfaces::Sensors::ROS::IMU(&nh, path, "boat_imu", 3);

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
