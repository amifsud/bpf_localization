
#define INTEGRATION_METHOD 1

#include "bpf_localization/dynamical_systems/dynamical_systems.hpp"

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
        TurtleBotDynamicalModel(const double dt              = 0.01,   // dt
                                const double wheels_radius   = 3.5e-2, // wheels radius
                                const double wheels_distance = 23e-2,  // wheels distance
                                Vector measures_noise_diams
                                    = Vector(TurtleBotDynamicalModel::measures_size, NaN),
                                Vector process_noise_diams
                                    = Vector(TurtleBotDynamicalModel::state_size, NaN))
            :DynamicalModel(state_size,           
                            control_size,         
                            measures_size,        
                            dt,                            
                            measures_noise_diams, 
                            process_noise_diams),
            wheels_radius_(wheels_radius),
            wheels_distance_(wheels_distance)
        {
            #if INTEGRATION_METHOD == 0
            configureGuarantedIntegration(RK4,      // integration method 
                                          1e-6,     // precision
                                          false,    // adaptative_timestep
                                          0.1);     // initial timestep
            #endif

            if(process_noise_diams == Vector(state_size_, NaN))
                // process noise diameters
            {
                process_noise_diams[0] = 1e-2;
                process_noise_diams[1] = 1e-2;
                process_noise_diams[2] = 1e-2;
            }

            if(measures_noise_diams == Vector(measures_size_, NaN))
                // measures noise diameters
            {
                measures_noise_diams[0] = 1e-2;
                measures_noise_diams[1] = 1e-2;
            }

            #if RESAMPLING_DIRECTION == 1
            normalization_values_.push_back(std::make_tuple(0, 1, 1.));
            normalization_values_.push_back(std::make_tuple(1, 1, 1.));
            normalization_values_.push_back(std::make_tuple(2, 1, 1.));
            #endif
        }

    protected:
        Function* getDynamicalModel(const IntervalVector* control, Variable* state)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            auto dynamical_model =            
                  new Function(*state, 
                        Return( wheels_radius_/2*(CONTROL(0)+CONTROL(1))*cos(STATE(2)),
                                wheels_radius_/2*(CONTROL(0)+CONTROL(1))*sin(STATE(2)),
                                wheels_radius_/wheels_distance_*(CONTROL(0)-CONTROL(1))));
            ROS_DEBUG_STREAM("set dynamical model end");
            return dynamical_model;
        }

        Function* getMeasuresModel(Variable* state)
        {
            ROS_DEBUG_STREAM("set measures model begin");
            auto measures_model =  
                  new Function(*state, Return(STATE(0),
                                              STATE(1)+STATE(2)));
            ROS_DEBUG_STREAM("set measures model end");
            return measures_model;
        }
};

