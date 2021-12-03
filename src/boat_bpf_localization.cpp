#define GUARANTED_RESAMPLING
#define UNIFORMLY_CHOOSEN_PAVING_INIT

#include <bpf_localization/boat_bpf_localization.hpp>
#include "bpf_localization/sensors.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "boat_bpf_localization");
    ros::NodeHandle nh;

    double pos = 1.;
    double vel = 0.1;
    double theta = 10./180.*3.14;

    BoatBPFLocalization boat(pos, vel, theta, false);
    IMUInterface        imu(&nh, "boat_imu", 3);

    IntervalVector control(INSDynamicalModel::control_size);

    while (ros::ok()) 
    {
        try
        {
            control = imu.getFirstIntervalValue();
            boat.IMUCallback(control);
            ROS_INFO_STREAM(control);
        }
        catch(int i) {}

        ros::spinOnce();
    }

    return 0;
}
