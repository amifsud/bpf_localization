
#define INTEGRATION_METHOD 1

#include "bpf_localization/dynamical_systems/dynamical_systems.hpp"

/** Turtlebot 2 **/

namespace dynamical_systems
{
    class TurtleBot: public dynamical_systems::DynamicalSystem
    {
        public:
            static const unsigned int state_size       = 3;         // state_size
            static const unsigned int control_size     = 2;         // control_size 
            static const unsigned int measures_size    = 2;         // measures_size

            const double wheels_radius_;
            const double wheels_distance_;
     
        public:
            TurtleBot(  const double dt = 0.01):
                dynamical_systems::DynamicalSystem( state_size,           
                                                    control_size,         
                                                    measures_size,        
                                                    dt)
            {
                #if INTEGRATION_METHOD == 0
                configureGuarantedIntegration(RK4,      // integration method 
                                              1e-6,     // precision
                                              false,    // adaptative_timestep
                                              0.1);     // initial timestep
                #endif

                #if RESAMPLING_DIRECTION == 1
                normalization_values_.push_back(std::make_tuple(0, 1, 1.));
                normalization_values_.push_back(std::make_tuple(1, 1, 1.));
                normalization_values_.push_back(std::make_tuple(2, 1, 1.));
                #endif
            }

        protected:
            Function* computeDynamicalModel(const IntervalVector* control, Variable* state)
            {
                ROS_DEBUG_STREAM("set dynamical model begin");
                auto dynamical_model =            
                      new Function(*state, 
                            Return( wheels_radius/2*(CONTROL(0)+CONTROL(1))*cos(STATE(2)),
                                    wheels_radius/2*(CONTROL(0)+CONTROL(1))*sin(STATE(2)),
                                    wheels_radius/wheels_distance*(CONTROL(0)-CONTROL(1))));
                ROS_DEBUG_STREAM("set dynamical model end");
                return dynamical_model;
            }

            Function* computeMeasuresModel(Variable* state)
            {
                ROS_DEBUG_STREAM("set measures model begin");
                auto measures_model =  
                      new Function(*state, Return(STATE(0),
                                                  STATE(1)+STATE(2)));
                ROS_DEBUG_STREAM("set measures model end");
                return measures_model;
            }
    };
} // namespace dynamical_systems
