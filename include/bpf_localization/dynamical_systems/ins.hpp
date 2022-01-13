
#define INTEGRATION_METHOD 1

#include "bpf_localization/dynamical_systems/dynamical_systems.hpp"

/*** Inertial Navigation System (INS) ***/

class INSDynamicalModel: public DynamicalModel
{
    public:
        static const unsigned int state_size       = 3*3+1; // state_size
        static const unsigned int control_size     = 2*3;   // control_size 
        static const unsigned int measures_size    = 3;     // measures_size

        double guz[3]={0.,0., -9.81};
	    Vector guz_;

    public:
        INSDynamicalModel(  const double dt              = NaN,  // dt
                            Vector measures_noise_diams
                                = Vector(INSDynamicalModel::measures_size, NaN),
                            Vector process_noise_diams
                                = Vector(INSDynamicalModel::state_size, NaN))
            :DynamicalModel(state_size,           
                            control_size,         
                            measures_size,        
                            dt,                            
                            measures_noise_diams, 
                            process_noise_diams),
            guz_(3, guz)
        {
            #if INTEGRATION_METHOD == 0
            configureGuarantedIntegration(HEUN,     // integration method 
                                          1e-4,     // precision
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

        Function* getMeasuresModel(Variable* state)
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

