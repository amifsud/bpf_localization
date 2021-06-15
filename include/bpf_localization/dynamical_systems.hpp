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

    public:
        DynamicalModel( unsigned int state_size, unsigned int control_size, 
                                unsigned int measures_size, double dt,
                                Vector measures_noise_diams, Vector process_noise_diams,
                                Method method, double precision)
            :dt_(dt), state_size_(state_size), 
             control_size_(control_size), measures_size_(measures_size), 
             measures_noise_diams_(measures_noise_diams),
             process_noise_diams_(process_noise_diams),
             integration_method_(method), precision_(precision),
             state(state_size)
        {
            ROS_ASSERT_MSG(state_size > 0, "State size has to be greater than 0");
        }
 
        IntervalVector applyDynamics(const IntervalVector& box, 
                                     IntervalVector& control, bool ivp = false)
        {
            if(ivp)
            {
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
                return dynamical_model_->eval_vector(box);
            }
        }

        IntervalVector applyMeasures(const IntervalVector& box)
        {
            return measures_model_->eval_vector(box);
        }

        const double& dt() const
        {
            return dt_;
        }

        const unsigned int& stateSize() const
        {
            return state_size_;
        }

        const unsigned int& controlSize() const
        {
            return control_size_;
        }

        const unsigned int& measuresSize() const
        {
            return measures_size_;
        }

        const Vector& processNoiseDiams() const
        {
            return process_noise_diams_;
        }

        const Vector& measuresNoiseDiams() const
        {
            return measures_noise_diams_;
        }

    protected:
        virtual void setDynamicalModel(const IntervalVector& control) = 0;
        virtual void setMeasuresModel (const IntervalVector& control) = 0;

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
        TurtleBotDynamicalModel(Vector measures_noise_diams
                                    = Vector(TurtleBotDynamicalModel::measures_size, NaN),
                                Vector process_noise_diams
                                    = Vector(TurtleBotDynamicalModel::state_size, NaN),
                                const double dt              = 1,      // dt
                                const Method method          = RK4,    // method       
                                const double precision       = 1e-6,   // precision
                                const double wheels_radius   = 3.5e-2, // wheels radius
                                const double wheels_distance = 23e-2)  // wheels distance
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
            if(process_noise_diams == Vector(state_size_, NaN))      // process noise diameters
            {
                process_noise_diams[0] = 1e-2;
                process_noise_diams[1] = 1e-2;
                process_noise_diams[2] = 1e-2;
            }

            if(measures_noise_diams == Vector(measures_size_, NaN)) // measures noise diameters
            {
                measures_noise_diams[0] = 1e-2;
                measures_noise_diams[1] = 1e-2;
            }
        }

    protected:
        void setDynamicalModel(const IntervalVector& control)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            dynamical_model_             
                = new Function(state, 
                        Return( wheels_radius_/2*(control[0]+control[1])*cos(state[2]),
                                wheels_radius_/2*(control[0]+control[1])*sin(state[2]),
                                wheels_radius_/wheels_distance_*(control[0]-control[1])));
            ROS_DEBUG_STREAM("set dynamical model end");
        }

        void setMeasuresModel(const IntervalVector& measures)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            measures_model_ 
                = new Function(state, Return( state[0],
                                              state[1]+state[2]));
            ROS_DEBUG_STREAM("set dynamical model end");
        }
};

#endif
