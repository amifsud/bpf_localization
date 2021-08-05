#include "ibex/ibex.h"
#include <ros/ros.h>
#include <math.h>

/** FIXME
 *
 *  - control in state to not reconstruct objects ?
 *  - specific dynamcal models witht there tests and examples are plugins ?
 *  - integration from IVP to define dynamical model by default (ibex finctions composition)
 *
 */

#ifndef DYNAMICAL_SYSTEMS
#define DYNAMICAL_SYSTEMS

#define NaN std::numeric_limits<double>::quiet_NaN()

using namespace ibex;

class DynamicalModel
{
    protected:
        double dt_; 

        // Process
        Function* dynamical_model_;
        Vector process_noise_diams_;
        unsigned int state_size_;
        unsigned int control_size_;
 
        // Measures
        Function* measures_model_;
        Vector measures_noise_diams_;
        unsigned int measures_size_;

        // Simulation with dynibex
        simulation* simu_;
        Method integration_method_;
        Variable  state;
        double precision_;
        bool ivp_;

    #if RESAMPLING_DIRECTION == 1
    public:
        std::vector<std::tuple<int, int, double>> normalization_values_;
    #endif

    public:
        DynamicalModel( unsigned int state_size, unsigned int control_size, 
                        unsigned int measures_size, double dt,
                        Vector measures_noise_diams, Vector process_noise_diams,
                        Method method, double precision, bool ivp)
            :dt_(dt), state_size_(state_size), 
             control_size_(control_size), measures_size_(measures_size),
             dynamical_model_(NULL),
             measures_model_(NULL),
             measures_noise_diams_(measures_noise_diams),
             process_noise_diams_(process_noise_diams),
             integration_method_(method), precision_(precision),
             state(state_size),
             ivp_(ivp)
        {
            ROS_ASSERT_MSG(state_size > 0, "State size has to be greater than 0");
        }
 
        IntervalVector applyDynamics(const IntervalVector& box, 
                                     const IntervalVector& control)
        {
            if(ivp_)
            {
                setIVPDynamicalModel(control);
                ivp_ode problem 
                    = ivp_ode(*dynamical_model_, 0.0, box);

                simu_ = new simulation(&problem, dt_, integration_method_, precision_);
                simu_->run_simulation();

                return simu_->get_last();
            }
            else
            {
                setDynamicalModel(control);
                return dynamical_model_->eval_vector(box);
            }
        }

        IntervalVector applyMeasures(const IntervalVector& box)
        {
            setParentMeasuresModel();
            return measures_model_->eval_vector(box);
        }

        const double& dt()                 const { return dt_; }
        const unsigned int& stateSize()    const { return state_size_; }
        const unsigned int& controlSize()  const { return control_size_; }
        const unsigned int& measuresSize() const { return measures_size_; }
        const Vector& processNoiseDiams()  const { return process_noise_diams_; }
        const Vector& measuresNoiseDiams() const { return measures_noise_diams_; }

    protected:
        virtual void setIVPDynamicalModel(const IntervalVector& control)
        {
            ROS_ASSERT_MSG(false, 
                    "IVP dynamical model not set, initialize it or use not IVP version");
        }

        virtual void setDynamicalModel   (const IntervalVector& control)
        {
            ROS_ASSERT_MSG(false, 
                    "dynamical model not set, initialize it or use IVP version");
        }

        virtual void setMeasuresModel()
        {
            ROS_ASSERT_MSG(false, "measures model not set");
        }

        void setParentMeasuresModel()
        {
            if(measures_model_ == NULL) setMeasuresModel();
        }

};

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
                                const bool   ivp             = false,  // IVP or not
                                const Method method          = RK4,    // method       
                                const double precision       = 1e-6,   // precision
                                Vector measures_noise_diams
                                    = Vector(TurtleBotDynamicalModel::measures_size, NaN),
                                Vector process_noise_diams
                                    = Vector(TurtleBotDynamicalModel::state_size, NaN))
            :DynamicalModel(state_size,           
                            control_size,         
                            measures_size,        
                            dt,                            
                            measures_noise_diams, 
                            process_noise_diams,  
                            method,                  
                            precision,
                            ivp),
            wheels_radius_(wheels_radius),
            wheels_distance_(wheels_distance)
        {
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
        void setIVPDynamicalModel(const IntervalVector& control)
        {
            ROS_DEBUG_STREAM("set IVP dynamical model begin");
            dynamical_model_             
                = new Function(state, 
                        Return( wheels_radius_/2*(control[0]+control[1])*cos(state[2]),
                                wheels_radius_/2*(control[0]+control[1])*sin(state[2]),
                                wheels_radius_/wheels_distance_*(control[0]-control[1])));
            ROS_DEBUG_STREAM("set IVP dynamical model end");
        }

        void setDynamicalModel(const IntervalVector& control)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            dynamical_model_             
                = new Function(state, 
                        Return(dt_*wheels_radius_/2*(control[0]+control[1])*cos(state[2])+state[0],
                               dt_*wheels_radius_/2*(control[0]+control[1])*sin(state[2])+state[1],
                               dt_*wheels_radius_/wheels_distance_*(control[0]-control[1])+state[2]
                               ));
            ROS_DEBUG_STREAM("set dynamical model end");
        }

        void setMeasuresModel()
        {
            ROS_DEBUG_STREAM("set measures model begin");
            measures_model_ 
                = new Function(state, Return( state[0],
                                              state[1]+state[2]));
            ROS_DEBUG_STREAM("set measures model end");
        }
};

class IMUDynamicalModel: public DynamicalModel
{
    public:
        static const unsigned int state_size       = 3*3+1;         // state_size
        static const unsigned int control_size     = 2*3;         // control_size 
        static const unsigned int measures_size    = control_size;         // measures_size
 
    public:
        IMUDynamicalModel(const double dt              = NaN,   // dt
                                const bool   ivp             = true,  // IVP or not
                                const Method method          = RK4,    // method       
                                const double precision       = 1e-6,   // precision
                                Vector measures_noise_diams
                                    = Vector(TurtleBotDynamicalModel::measures_size, NaN),
                                Vector process_noise_diams
                                    = Vector(TurtleBotDynamicalModel::state_size, NaN))
            :DynamicalModel(state_size,           
                            control_size,         
                            measures_size,        
                            dt,                            
                            measures_noise_diams, 
                            process_noise_diams,  
                            method,                  
                            precision,
                            ivp)
        {
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
        void setIVPDynamicalModel(const IntervalVector& control)
        {
            ROS_DEBUG_STREAM("set IVP dynamical model begin");

            /*Variable  quaternion(4);

            Function t2 = Function(quaternion, quaternion[3]*quaternion[0]);
            Function t3 = Function(quaternion, quaternion[3]*quaternion[1]);
            Function t4 = Function(quaternion, quaternion[3]*quaternion[2]);
            Function t5 = Function(quaternion, -quaternion[0]*quaternion[0]);
            Function t6 = Function(quaternion, quaternion[0]*quaternion[1]);
            Function t7 = Function(quaternion, quaternion[0]*quaternion[2]);
            Function t8 = Function(quaternion, -quaternion[1]*quaternion[1]);
            Function t9 = Function(quaternion, quaternion[1]*quaternion[2]);
            Function t10 = Function(quaternion, -quaternion[2]*quaternion[2]);

            Function rotate(quaternion, Return( 2*( (t8(quaternion) + t10(quaternion))
                                                        *vector_in[0] 
                                                  + (t6(quaternion) -  t4(quaternion))
                                                        *vector_in[1] 
                                                  + (t3(quaternion) + t7(quaternion))
                                                        *vector_in[2] ) + vector_in[0],
                                                2*( (t4(quaternion) +  t6(quaternion))
                                                        *vector_in[0] 
                                                  + (t5(quaternion) + t10(quaternion))
                                                        *vector_in[1] 
                                                  + (t9(quaternion) - t2(quaternion))
                                                        *vector_in[2] ) + vector_in[1],
                                                2*( (t7(quaternion) -  t3(quaternion))
                                                        *vector_in[0] 
                                                  + (t2(quaternion) +  t9(quaternion))
                                                        *vector_in[1] 
                                                  + (t5(quaternion) + t8(quaternion))
                                                        *vector_in[2] ) + vector_in[2]));
            
            Function select_quaternion(state, Return(state[0], state[1], state[2], state[3]));
            Function rotated_accelero = rotate(select_quaternion, control.subvector(3,5));
            Function rotated_gyro = rotate(select_quaternion, control.subvector(0,2));
            
            double _guz[3]={0.,0., -9.81};
	        Vector guz(3,_guz);
            dynamical_model_             
                = new Function(state, 
                        Return(state[1], 
                               rotated_accelero-guz, 
                               rotated_gyro);*/
            ROS_DEBUG_STREAM("set IVP dynamical model end");
        }
};

#endif
