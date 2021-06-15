#include <ros/ros.h>
#include <math.h>

#define NaN std::numeric_limits<double>::quiet_NaN()

/** Turtlebot 2 **/

class TurtleBotDynamicalModel: public DynamicalModel
{
    public:
        static const unsigned int state_size       = 3;         // state_size
        static const unsigned int control_size     = 2;         // control_size 
        static const unsigned int measures_size    = 2;         // measures_size

        const double wheels_radius_;
        const double wheels_distance_;
 
    public:
        TurtleBotDynamicalModel(Vector measures_noise_diams
                                    = Vector(TurtleBotDynamicalModel::measures_size, NaN),
                                Vector process_noise_diams
                                    = Vector(TurtleBotDynamicalModel::state_size, NaN),
                                double dt              = 1,      // dt
                                Method method          = RK4,    // method       
                                double precision       = 1e-6,   // precision
                                double wheels_radius   = 3.5e-2, // wheels radius
                                double wheels_distance = 23e-2)  // wheels distance
            :DynamicalModel(state_size,           
                            control_size,         
                            measures_size,        
                            dt,                            
                            measures_noise_diams, 
                            process_noise_diams,  
                            method,                  
                            precision),
            wheels_radius_(wheels_radius),
            wheels_distance_(wheels_distance)
        {
            if(process_noise_diams == Vector(state_size_, 0.0))      // process noise diameters
            {
                process_noise_diams[0] = 1e-2;
                process_noise_diams[1] = 1e-2;
                process_noise_diams[2] = 1e-2;
            }

            if(measures_noise_diams == Vector(measures_size_, 0.0)) // measures noise diameters
            {
                measures_noise_diams[0] = 1e-2;
                measures_noise_diams[1] = 1e-2;
            }
        }

    protected:
        void setDynamicalModel(IntervalVector& control)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            dynamical_model_             
                = new Function(state, 
                        Return( Interval(1.),
                                Interval(1.),
                                control[0]));
            ROS_DEBUG_STREAM("set dynamical model end");
        }

        void setMeasuresModel(IntervalVector& measures)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            measures_model_ 
                = new Function(state, Return( state[0],
                                              state[1]+state[2]));
            ROS_DEBUG_STREAM("set dynamical model end");
        }
};

