#define GUARANTED_RESAMPLING
#define UNIFORMLY_CHOOSEN_PAVING_INIT

#include <bpf_localization/boat_bpf_localization.hpp>
#include "bpf_localization/interfaces/imu.hpp"
#include "bpf_localization/interfaces/gps.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "boat_bpf_localization");
    ros::NodeHandle nh;

    double pos = 1.;
    double vel = 0.1;
    double theta = 10./180.*3.14;

    std::string path = ros::package::getPath("bpf_localization");
    path += "/data/calibrations/";

    bpf::BoatBPFLocalization boat_localization(pos, vel, theta, false);
    Interfaces::Sensors::ROS::IMU imu(&nh, path, "boat_imu", 3);
    Interfaces::Sensors::ROS::GPS gps(&nh, path, "boat_gps", 3);

    IntervalVector control(dynamical_systems::INS::control_size);
    IntervalVector position(3);

    unsigned int u = 0;
    while (ros::ok()) 
    {
        try
        {
            control = imu.getFirstIntervalValue();
            boat_localization.IMUCallback(control);
            //ROS_INFO_STREAM("Control = " << control);
        }
        catch(int i) {}

        if(u >= 5)
        {
            try
            {
                position = gps.getFirstIntervalValue();
                //boat_localization.GPSCallback(position);
                ROS_INFO_STREAM("Position = " << position);
                u = 0;
            }
            catch(int i) {}
        }

        u++;

        ros::spinOnce();
    }

    return 0;
}
