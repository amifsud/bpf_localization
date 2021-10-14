#define WO_IBEX
#include "bpf_localization/utils.hpp"

#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>

class IMU
{
    public:
        ros::Publisher  imu_pub_;
        ros::Subscriber imu_sub_;

        sensor_msgs::Imu imu_msg_;

    public:
        IMU(ros::NodeHandle* nh)
        {
            imu_pub_ = nh->advertise<sensor_msgs::Imu>("imu_out", 50);
            imu_sub_ = nh->subscribe("imu_in", 50, &IMU::callback_imu, this);
        }

        void callback_imu(const sensor_msgs::Imu& imu_in) 
        {
            imu_msg_ = imu_in;
            imu_msg_.header.frame_id = "imu_calibrated";
            
            imu_msg_.angular_velocity.x    = imu_in.angular_velocity.x;
            imu_msg_.angular_velocity.y    = imu_in.angular_velocity.y;
            imu_msg_.angular_velocity.z    = imu_in.angular_velocity.z;
            imu_msg_.linear_acceleration.x = imu_in.linear_acceleration.x;
            imu_msg_.linear_acceleration.y = imu_in.linear_acceleration.y;
            imu_msg_.linear_acceleration.z = imu_in.linear_acceleration.z;

            imu_pub_.publish(imu_msg_);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_interface");
    ros::NodeHandle nh;

    IMU imu = IMU(&nh);

    ros::Rate r(5.0);
    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
