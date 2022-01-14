
#define INTEGRATION_METHOD 1

#include "bpf_localization/dynamical_systems/dynamical_systems.hpp"

/*** Inertial Navigation System (INS) ***/

namespace dynamical_systems
{
    class INS: public DynamicalSystem
    {
        public:
            static const unsigned int state_size       = 3*3+1; // state_size
            static const unsigned int control_size     = 2*3;   // control_size 
            static const unsigned int measures_size    = 3;     // measures_size

            double guz[3]={0.,0., -9.81};
            Vector guz_;

        public:
            INS(const double dt = NaN):
                DynamicalSystem(state_size, control_size, measures_size, dt),
                guz_(3, guz)
            {
                #if INTEGRATION_METHOD == 0
                configureGuarantedIntegration(HEUN,     // integration method 
                                              1e-4,     // precision
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
                        ReturnIMU(0.5*(-STATE(1)*CONTROL(0)-STATE(2)*CONTROL(1)-STATE(3)*CONTROL(2)),
                                0.5*( STATE(0)*CONTROL(0)-STATE(3)*CONTROL(1)+STATE(2)*CONTROL(2)),
                                0.5*( STATE(3)*CONTROL(0)+STATE(0)*CONTROL(1)-STATE(1)*CONTROL(2)),
                                0.5*(-STATE(2)*CONTROL(0)+STATE(1)*CONTROL(1)+STATE(0)*CONTROL(2)),
                                STATE(7),
                                STATE(8),
                                STATE(9),
                                2*( (-STATE(2)*STATE(2) - STATE(3)*STATE(3))*CONTROL(3) 
                                  + ( STATE(1)*STATE(2) - STATE(0)*STATE(3))*CONTROL(4) 
                                  + ( STATE(0)*STATE(2) + STATE(1)*STATE(3))*CONTROL(5) ) 
                                        + CONTROL(3) ,
                                2*( ( STATE(0)*STATE(3) + STATE(1)*STATE(2))*CONTROL(3) 
                                  + (-STATE(1)*STATE(1) - STATE(3)*STATE(3))*CONTROL(4) 
                                  + ( STATE(2)*STATE(3) - STATE(0)*STATE(1))*CONTROL(5) ) 
                                        + CONTROL(4) ,
                                2*( ( STATE(1)*STATE(3) - STATE(0)*STATE(2))*CONTROL(3) 
                                  + ( STATE(0)*STATE(1) + STATE(2)*STATE(3))*CONTROL(4) 
                                  + (-STATE(1)*STATE(1) - STATE(2)*STATE(2))*CONTROL(5) ) 
                                        + CONTROL(5) - guz[2]));

                ROS_DEBUG_STREAM("set dynamical model end");
                return dynamical_model;
            }

            Function* computeMeasuresModel(Variable* state)
            {
                ROS_DEBUG_STREAM("set measures model begin");

                auto measures_model = 
                        new Function(*state, Return( STATE(4),
                                                     STATE(5),
                                                     STATE(6)));

                ROS_DEBUG_STREAM("set measures model end");
                return measures_model;
            }
        };
}
