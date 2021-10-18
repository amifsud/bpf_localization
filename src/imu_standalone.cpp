#define LOG_LEVEL Info

#include "bpf_localization/sensors.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_standalone");
    ros::NodeHandle nh;

    auto imu = IMUInterface(&nh, "boat_imu", 3);

    ros::Rate r(1./5);

    std::deque<IntervalVector> data;

    r.sleep();

    data = imu.getIntervalData();
    ROS_INFO_STREAM("SIZE = " << data.size());
    
    ros::spinOnce();

    r.sleep();

    data = imu.getIntervalData();
    ROS_INFO_STREAM("SIZE = " << data.size());
    for(auto it = data.begin(); it != data.end(); it++)
        ROS_INFO_STREAM(*it);
    ros::spinOnce();

    /*while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }*/
}
