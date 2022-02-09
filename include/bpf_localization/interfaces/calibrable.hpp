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
     * \brief Accumulating data, determining lower/upper bounds and writing results to a file.
     *
     * The calibrationDataFormat method has to be implemented in derived classes to 
     * define which data are collected.
     *
     * For each calibration_data_format_ line defined in calibrationDataFormat, 
     * we determine and log:
     *  - lower bound of accumulated data
     *  - upper bound of accumulated data
     *  - mid of the bounds
     *  - mean of the data
     *
     *  From those values, the diameter (half_diameter) of the calibration_data_format_ 
     *  line data is computed and can be used in derivded classes.
     *
     *  Those values can be imported from a file, updated from new data, 
     *  and append in the file.
     *
     *  The file writing format is compatible for importation in pandas python module. 
     *  See example of this in python/bpf_localization/test_pandas.py 
     *
     */
    class Calibrable
    {
        public:
            /*! Calibrable( std::string path, std::string name, 
                            unsigned int size, unsigned int decimal):
                            calibration_(false), name_(name), nb_(0), 
                            precision_(pow(10, decimal)), 
                            size_(size), sum_(Vector(size, 0.)), 
                            half_diameters_(Vector(size, 0.)),
                            calibration_file_(path + name + ".csv")
             *
             *  \brief Constructor
             *
             *  The values determined from data are defined here. That is:
             *      - lower bound of accumulated data
             *      - upper bound of accumulated data
             *      - mid of the bounds
             *      - mean of the data
             *
             *  \param path the path of the calibration file
             *  \param name the name of the calibration file (without extension)
             *  \param size the number of data accumulated (have to be coherent 
             *          with the size of calibration_data_format_ 
             *  \param decimal specify the number of decimal to keep in data 
             *          (+/-1e^(decimal) is added to bounds to increase the interval size)
             *
             */
            Calibrable( std::string path, std::string name, 
                        unsigned int size, unsigned int decimal):
                calibration_(false), nb_(0), 
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

            /*! bool isCalibrating(){ return calibration_; }
             *
             *  \brief Retun if the Calibrable is currently update the bounds from data or not
             *
             *  \result boolean that say if Calibrable is calobrating
             */
            bool isCalibrating(){ return calibration_; }

        protected:
            /*! virtual void calibrationDataFormat()
             *
             *  \brief Determine the format of the data that are accumulated
             *
             *  This is a virtual method that has to be implemented in you 
             *  application derived class
             *
             */
            virtual void calibrationDataFormat() = 0;

            /*! void computeHalfDiameters()
             *
             *  \brief Compute half diameter of the interval from bounds
             *
             *  Update data_map_ and half_diameters_ accordingly.
             *
             *  half_diameters_ correspond to the half of the diameter of the interval
             *  we have to generate around every data. This interval is inflated with i
             *  a factor 2 because for each incoming data, we don't now where it is 
             *  situated in the interval.
             */
            void computeHalfDiameters()
            {
                data_map_.at("mid") = 0.5*(data_map_.at("ub") + data_map_.at("lb"));
                // half_diameter = 2*((ub-lb)/2)
                half_diameters_ = data_map_.at("ub") - data_map_.at("lb");
            }

            /*! \name Calibration file gestion */

            ///@{
            /*! void splitToDouble( std::string* line, std::string* format,
             *                   std::vector<double>* values)
             *
             *  \param line the string to split
             *  \param format a string to delete from line begining 
             *          (usually got from calibration_data_fornat_)
             *  \param values the output values (double) extracted from line
             */
            void splitToDouble( std::string* line, std::string* format,
                                std::vector<double>* values)
            {
                values->clear();
                std::string number;
                line->erase(0, format->size());
                std::stringstream stream(*line);
                while(getline(stream, number, ',')) values->push_back(std::stod(number));
            }

            /*! void loadFromFile()
             *
             *  \brief load interval data from the calibration file
             *
             *  If the calibration file contains several calibration data, i
             *  we take the encapsulating interval.
             *
             */
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

            /*! void writeToFile()
             *
             *  \brief Update the calibration file with the current interval values
             *
             *  If the calibration file don't exist it is created.
             *
             *  If the calibration file is empty it is initialized according to the
             *  calibration_data_format_ and data_map_ values.
             *
             */
            void writeToFile()
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
            ///@}

            /*! \name Calibration start and stop */

            ///@{
            /*! void startCalibration(unsigned int init_time, unsigned int until)
             *
             *  \brief Start a calibration
             *
             *  Initialize all usefull attributes and clear the buffer
             *
             *  \param init_time the time when the calibration is started
             *  \param duration the duration of the calibration
             */
            void startCalibration(unsigned int init_time, unsigned int duration)
            {
                calibration_ = true;
                sum_ = Vector(size_, 0.);
                nb_ = 0.;
                data_.clear();
                init_time_ = init_time;
                duration_ = duration;
            }

            /*! void stopCalibration()
             *
             *  \brief Stop the calibration
             *
             *  This update one more time the interval values and update the calibration file
             *
             */
            void stopCalibration()
            {
                ROS_INFO_STREAM("Stoppping calibration");
                calibration_ = false;
                update();
                writeToFile();
            }
            ///@}

            /*! \name Update from new data */

            ///@{
            /*! void update()
             *
             *  \brief Update interval values with the accumulated data in data_ buffer
             *
             *  Use the data in data_ to update all interval values
             *  (lb, ub, mid, mean, half_diameters) then clear the data_ buffer
             *
             */
            void update()
            {
                for(auto vect = data_.begin(); vect !=data_.end(); vect++)
                {
                    for(unsigned int i = 0; i < size_; ++i)
                    {
                        if(vect->operator[](i) > data_map_.at("ub")[i] - 1./precision_)
                        {
                            data_map_.at("ub")[i] 
                                = roundf(vect->operator[](i) * precision_) 
                                    / precision_ + 1./precision_;
                        }

                        if(vect->operator[](i) < data_map_.at("lb")[i] + 1./precision_) 
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

            /*! void feed(const Vector& vect, unsigned int time)
             *
             *  \brief Feed the data_ buffer
             *
             *  If the calibration duration is reached, stop the calibration
             *
             */
            void feed(const Vector& vect, unsigned int time)
            {
                ROS_INFO_STREAM("Calibrating...");

                data_.push_back(Vector(vect));
                update();
                time_ = time - init_time_; 

                if(time_ >= duration_) stopCalibration();
            }
            ///@}

        protected:
            /*! \name Calibration time gestion */

            ///@{
            /*! Is the calibrable calibrating or not */
            bool calibration_;
            /*! Init time of the calibration */
            unsigned int init_time_;
            /*! Duration of the calibration */
            unsigned int duration_;
            /*! Current time in the calibration */
            unsigned int time_;
            ///@}

            /*! \name Accumulated data */

            ///@{
            /*! Size of the data Vector to accumulate */
            unsigned int size_;
            /*! Accumulated data */
            std::vector<Vector> data_;
            /*! Format of the accumulated data */
            std::vector<std::string> calibration_data_format_;
            ///@}

            /*! \name Computed interval values */

            ///@{
            /*! Interval values computed from the accumulated data */
            std::map<std::string, Vector> data_map_;
            /*! Half diameter used in derived classes to encapsulate new data */
            Vector half_diameters_;
            /*! Decimal number we keep in input data to update interval values */
            double precision_;
            /*! Sum of accumulated data to compute mean using #nb_*/
            Vector sum_;
            /*! Number of accumulated data to compute mean using #sum_*/
            double nb_;
            ///@}

            /*! \name Calibration file */

            ///@{
            /*! Iterface with the calibration file */
            File calibration_file_;
            ///@}
    };
} // Namespace Interfaces

#endif
