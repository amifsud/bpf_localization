#include "ibex/ibex.h"
#include <ros/ros.h>
#include <math.h>

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
 
        IntervalVector applyDynamics(const IntervalVector& box, const IntervalVector& control)
        {
            if(ivp_)
            {
                setIVPDynamicalModel(control);
                ivp_ode problem 
                    = ivp_ode(*dynamical_model_, 0.0, IntervalVector(box.size()));
                IntervalVector box_tmp(box);
                problem.yinit = &box_tmp;

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
                                const Method method          = RK4,    // method       
                                const double precision       = 1e-2,   // precision
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
                            false),                                    // IVP or not
            wheels_radius_(wheels_radius),
            wheels_distance_(wheels_distance)
        {
            if(process_noise_diams == Vector(state_size_, NaN))        // process noise diameters
            {
                process_noise_diams[0] = 1e-2;
                process_noise_diams[1] = 1e-2;
                process_noise_diams[2] = 1e-2;
            }

            if(measures_noise_diams == Vector(measures_size_, NaN))    // measures noise diameters
            {
                measures_noise_diams[0] = 1e-2;
                measures_noise_diams[1] = 1e-2;
            }
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
                               dt_*wheels_radius_/wheels_distance_*(control[0]-control[1]+state[2])
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

#endif
