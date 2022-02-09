
/** FIXME: 
 * Make unit tests for the file
**/

/**
 * \file   interfaces.hpp
 * \brief  Interfaces
 * \author Alexis Mifsud
 * \date   2022 January
 */

#ifndef CALIBRABLE
#define CALIBRABLE

#include "bpf_localization/utils.hpp"

/*! Interfaces
 *
 *  \brief Generic classes for interfaces
 *
 *  **For now only Sensors are implemented**
 *
 */
namespace Interfaces
{
    /*! \class Calibrable
     *
     *
     *
     */
    class Calibrable
    {
        public:
            Calibrable( std::string path, std::string name, 
                        unsigned int size, unsigned int decimal):
                calibration_(false), name_(name), nb_(0), 
                precision_(pow(10, decimal)), 
                size_(size), sum_(Vector(size, 0.)), 
                half_diameters_(Vector(size, 0.)),
                calibration_file_(path + name + ".csv")
            {
                data_map_.emplace("lb",   Vector(size, 1e7));
                data_map_.emplace("ub",   Vector(size, -1e7));
                data_map_.emplace("mid",  Vector(size, 1e7));
                data_map_.emplace("mean", Vector(size, 1e7));
            }

            bool isCalibrating(){ return calibration_; }

        protected:
            virtual void calibrationDataFormat() = 0;

            void splitToDouble(  std::string* line, std::string* format,
                                std::vector<double>* values)
            {
                values->clear();
                std::string number;
                line->erase(0, format->size());
                std::stringstream stream(*line);
                while(getline(stream, number, ',')) values->push_back(std::stod(number));
            }

            void computeHalfDiameters()
            {
                data_map_.at("mid") = 0.5*(data_map_.at("ub") + data_map_.at("lb"));
                // half_diameter = 2*((ub-lb)/2)
                half_diameters_ = data_map_.at("ub") - data_map_.at("lb");
            }

            void loadFromFile()
            {
                calibrationDataFormat();

                std::string line, format;
                std::vector<double> values;
                std:vector<std::string> lines;

                if(!calibration_file_.isEmpty())
                {
                    calibration_file_.read(&lines);

                    for(auto u = 0; u < calibration_data_format_.size()-1; ++u)
                    {
                        line   = lines[u*4+1];
                        format = calibration_data_format_[u+1]+",lb,";
                        splitToDouble(&line, &format, &values);
                        data_map_.at("lb")[u] 
                            = *std::min_element(values.begin(), values.end());

                        line   = lines[u*4+2];
                        format = calibration_data_format_[u+1]+",ub,";
                        splitToDouble(&line, &format, &values);
                        data_map_.at("ub")[u] 
                            = *std::max_element(values.begin(), values.end());
                    }

                    computeHalfDiameters();
                }

                calibration_data_format_.clear();
            }

            void startCalibration(unsigned int init_time, unsigned int until)
            {
                calibration_ = true;
                sum_ = Vector(size_, 0.);
                nb_ = 0.;
                data_.clear();
                init_time_ = init_time;
                until_ = until;
            }

            void update()
            {
                for(auto vect = data_.begin(); vect !=data_.end(); vect++)
                {
                    for(unsigned int i = 0; i < size_; ++i)
                    {
                        if(vect->operator[](i) > data_map_.at("ub")[i] + 1./precision_)
                        {
                            data_map_.at("ub")[i] 
                                = roundf(vect->operator[](i) * precision_) 
                                    / precision_ + 1./precision_;
                        }

                        if(vect->operator[](i) < data_map_.at("lb")[i] - 1./precision_) 
                        {
                            data_map_.at("lb")[i] 
                                = roundf(vect->operator[](i) * precision_) 
                                    / precision_ - 1./precision_;
                        }
                    }

                    sum_ += *vect;
                    nb_ += 1.;
                }

                computeHalfDiameters();
                data_map_.at("mean") = 1./nb_*sum_;
                data_.clear();
            }

            void feed(const Vector& vect, unsigned int time)
            {
                ROS_INFO_STREAM("Calibrating...");

                data_.push_back(Vector(vect));
                update();
                time_ = time - init_time_; 

                if(time_ >= until_) stopCalibration();
            }

            void stopCalibration()
            {
                ROS_INFO_STREAM("Stoppping calibration");
                calibration_ = false;
                update();
                writeCalibrationFile();
            }

            void writeCalibrationFile()
            {
                ROS_DEBUG_STREAM("Begin to write calibration file");

                calibrationDataFormat();

                std:vector<std::string> lines;

                // Initialize if empty
                if(calibration_file_.isEmpty())
                {
                    lines.push_back("vector,component,data");
                    for(auto u = 1; u < calibration_data_format_.size(); ++u)
                    {
                        for(auto it = data_map_.begin(); it != data_map_.end(); ++it)
                        {
                            lines.push_back(
                                calibration_data_format_[u] + "," + it->first);
                        }
                    }
                }
                else
                {
                    calibration_file_.read(&lines);
                }

                // Update calibration data
                lines.operator[](0) += "," + to_string(ros::Time::now().toSec());
                unsigned int u;
                for(auto i = 0; i < size_; ++i)
                {
                    u = 1;
                    for(auto it = data_map_.begin(); it != data_map_.end(); ++it)
                    {
                        lines.operator[](i*data_map_.size()+u) 
                            += ","+to_string(it->second.operator[](i));
                        u += 1;
                    }
                }

                calibration_file_.write(&lines);

                calibration_data_format_.clear();
                ROS_DEBUG_STREAM("End to write calibration file");
            }

        protected:
            unsigned int size_;
            bool calibration_;

            File calibration_file_;
            std::string name_;

            std::map<std::string, Vector> data_map_;
            std::vector<std::string> calibration_data_format_;

            std::vector<Vector> data_;
            Vector half_diameters_;
            Vector sum_;
            double nb_;

            unsigned int init_time_;
            unsigned int until_;
            unsigned int time_;

            double precision_;

    };
} // Namespace Interfaces

#endif
