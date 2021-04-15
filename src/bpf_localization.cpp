#include <bpf_localization/box_particle_filter.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) 
{

    BoxParticleFilter bpf;
    ROS_INFO_STREAM("a=" << bpf.get());

    ros::init(argc, argv, "bpf_localization");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
