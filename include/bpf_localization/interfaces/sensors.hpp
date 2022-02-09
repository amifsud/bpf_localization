
/** FIXME: 
 * Make unit tests for the file
**/

/**
 * \file   interfaces.hpp
 * \brief  Interfaces
 * \author Alexis Mifsud
 * \date   2022 January
 */

#ifndef SENSORS
#define SENSORS

#include <ros/package.h>
#include <message_filters/subscriber.h>

#include "bpf_localization/interfaces/calibrable.hpp"
#include "bpf_localization/StartCalibration.h"
#include "bpf_localization/GetDiameters.h"

namespace Interfaces
{
    /*! Sensors
     *  
     *  \brief Sensors classes
     *
     */
    namespace Sensors
    {
        class SensorInterface: public Calibrable
        {
            public:
                SensorInterface(std::string path, std::string name, 
                                unsigned int size, unsigned int decimal):
                    Calibrable(path, name, size, decimal)
                {
                }

                std::deque<IntervalVector> getIntervalData()
                {
                    std::deque<IntervalVector> buffer(buffer_);
                    buffer_.clear();
                    return buffer;
                }

                IntervalVector getFirstIntervalValue()
                {
                    if(!buffer_.empty())
                    {
                        auto interval = IntervalVector(buffer_.front());
                        buffer_.pop_front();
                        return interval;
                    }
                    else
                    {
                        throw 1;
                    }
                }

            protected:
                IntervalVector intervalFromVector(const Vector& data)
                {
                    IntervalVector interval(size_);
                    if(half_diameters_.max() < 1e-7) loadFromFile();

                    for(auto i = 0; i < size_; ++i)
                    {
                        interval[i] = Interval( data[i]-half_diameters_[i],
                                                data[i]+half_diameters_[i]);
                    }

                    return interval;
                }

                void feed(const Vector& data, unsigned int time)
                {
                    buffer_.push_back(intervalFromVector(data));
                    if(isCalibrating()) Calibrable::feed(data, time);
                }

            protected:
                std::deque<IntervalVector> buffer_;
        };

        /*! ROS
         *
         *  \brief ROS Interface for Sensors
         *
         */
        namespace ROS
        {
            class ROSInterface: virtual public SensorInterface
            {
                public:
                    ROSInterface(ros::NodeHandle* nh, std::string path, std::string name, 
                        unsigned int size, unsigned int decimal):
                        SensorInterface(path, name, size, decimal),
                        tmp_(Vector(size, 0.))
                    {
                        start_calibration_server_ 
                            = nh->advertiseService(name + "_start_calibration", 
                                    &ROSInterface::startCalibration, this);

                        stop_calibration_server_ 
                            = nh->advertiseService(name + "_stop_calibration", 
                                    &ROSInterface::stopCalibration, this);

                        diameters_server_ 
                            = nh->advertiseService( name + "_diameters", 
                                                    &ROSInterface::getDiameters, this);
                    }

                    bool getDiameters(bpf_localization::GetDiameters::Request  &req,
                                      bpf_localization::GetDiameters::Response &res)
                    {
                        if(half_diameters_.max() < 1e-7) loadFromFile();
                        for(auto i = 0; i < size_; ++i)
                            res.diameters.push_back(2*half_diameters_[i]);
                    }

                protected:
                    bool startCalibration(bpf_localization::StartCalibration::Request  &req,
                                          bpf_localization::StartCalibration::Response &res)
                    {
                        ROS_INFO_STREAM("Start calibration");
                        unsigned int init_time = (unsigned int)(ros::Time::now().toSec()); 
                        unsigned int until 
                            = (unsigned int)(ros::Duration(req.duration, 0).toSec());
                        Calibrable::startCalibration(init_time, until);
                        return true;
                    }

                    bool stopCalibration(bpf_localization::GetDiameters::Request  &req,
                                         bpf_localization::GetDiameters::Response &res)
                    {
                        Calibrable::stopCalibration();
                        for(auto i = 0; i < size_; ++i)
                            res.diameters.push_back(2*half_diameters_[i]);
                        return true;
                    }

                protected:
                    ros::ServiceServer start_calibration_server_;
                    ros::ServiceServer stop_calibration_server_;

                    ros::ServiceServer diameters_server_;
                    ros::Subscriber sub_;
                    ros::Publisher pub_;

                    Vector tmp_;
            };

        }
    }
}

#endif
