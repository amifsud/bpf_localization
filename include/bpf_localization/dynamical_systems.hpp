/** FIXME
 *
 *  - control in state to not reconstruct objects ?
 *  - specific dynamcal models witht there tests and examples are plugins ?
 *  - integration from IVP to define dynamical model by default (ibex finctions composition)
 *
 */

#ifndef DYNAMICAL_SYSTEMS
#define DYNAMICAL_SYSTEMS

/*** Integrations methods ***/
//  * guaranted with Dynibex IVPs                       : 0
//  * Non guaranted interval integration                : 1
//  * Non guaranted integration with affine arithmetic  : 2

#ifndef INTEGRATION_METHOD
    #define INTEGRATION_METHOD 1
#endif

#include "bpf_localization/utils.hpp"

using namespace ibex;

#define STATE(x) state->operator[](x)
#define CONTROL(x) control->operator[](x)

class DynamicalModel
{
    protected:
        double dt_; 

        // Process
        Vector process_noise_diams_;
        unsigned int state_size_;
        unsigned int control_size_;
 
        // Measures
        Vector measures_noise_diams_;
        unsigned int measures_size_;

        // Simulation with dynibex
        Method integration_method_;
        double precision_;
        bool guaranted_;
        bool adaptative_timestep_;
        double h_;

    #if RESAMPLING_DIRECTION == 1
    public:
        std::vector<std::tuple<int, int, double>> normalization_values_;
    #endif

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
             adaptative_timestep_(false)
        {
            ROS_ASSERT_MSG(state_size > 0, "State size has to be greater than 0");
        }
 
        IntervalVector applyDynamics(const IntervalVector& box, 
                                     const IntervalVector& control)
        {
            //assert_ready();
            Variable state(state_size_);
            IntervalVector result(state_size_, Interval(0., 0.));
            Function* dynamical_model = getDynamicalModel(&control, &state);

            #if INTEGRATION_METHOD == 0
                ivp_ode problem = ivp_ode(*dynamical_model, 0.0, box);

                simulation simu(&problem, dt_, integration_method_, precision_, h_);
                simu.run_simulation();

                if(adaptative_timestep_) 
                    h_ = simu.list_solution_g.back().time_j.diam();
                result = simu.get_last();
            #elif INTEGRATION_METHOD == 1
                AF_fAFFullI::setAffineNoiseNumber (2);
                Affine2Vector result_affine(box);

                Affine2Vector k1 
                    = dynamical_model->eval_affine2_vector(result_affine);
                Affine2Vector k2 
                    = dynamical_model->eval_affine2_vector(result_affine+dt_*k1);
                result_affine = result_affine + (0.5*dt_)*(k1+k2);
                result_affine.compact();

                result = result_affine.itv();
            #elif INTEGRATION_METHOD == 2
                IntervalVector k1 = dynamical_model->eval_vector(box);
                IntervalVector k2 = dynamical_model->eval_vector(box+dt_*k1);
                result = box + (0.5*dt_)*(k1+k2);
            #else 
                ROS_ASSERT_MSG(false && "Wrong integration method in preprocessor configuration");
            #endif

            delete dynamical_model;
            return result;
        }

        IntervalVector applyMeasures(const IntervalVector& box)
        {
            //assert_ready();
            Variable state(state_size_);
            Function* measures_model = getMeasuresModel(&state);
            return measures_model->eval_vector(box);
        }

        const double& dt()                 const { return dt_; }
        const unsigned int& stateSize()    const { return state_size_; }
        const unsigned int& controlSize()  const { return control_size_; }
        const unsigned int& measuresSize() const { return measures_size_; }
        const Vector& processNoiseDiams()  const { return process_noise_diams_; }
        const Vector& measuresNoiseDiams() const { return measures_noise_diams_; }

    protected:
        virtual Function* getDynamicalModel
            (const IntervalVector* control, Variable* state) = 0;
        virtual Function* getMeasuresModel(Variable* state) = 0;

        void assert_ready()
        {
            Variable state(state_size_);
            IntervalVector control(state_size_, Interval(0.,0.));

            Function* dynamical_model = getDynamicalModel( &control, &state);

            assert(dynamical_model != NULL && "dynamical_model_ not set");

            dynamical_model = getMeasuresModel(&state);
            assert(dynamical_model != NULL  && "measures_model_ not set");

            assert(state_size_ != 0         && "state_size not set");
            assert(control_size_ != 0       && "control_size not set");
            assert(measures_size_ != 0      && "measures_size not set");
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
                            precision),
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
        Function* getDynamicalModel(const IntervalVector* control, Variable* state)
        {
            ROS_DEBUG_STREAM("set IVP dynamical model begin");
            auto dynamical_model =            
                  new Function(*state, 
                        Return( wheels_radius_/2*(CONTROL(0)+CONTROL(1))*cos(STATE(2)),
                                wheels_radius_/2*(CONTROL(0)+CONTROL(1))*sin(STATE(2)),
                                wheels_radius_/wheels_distance_*(CONTROL(0)-CONTROL(1))));
            ROS_DEBUG_STREAM("set IVP dynamical model end");
            return dynamical_model;
        }

        Function* getMeasuresModel(Variable* state)
        {
            ROS_DEBUG_STREAM("set measures model begin");
            auto measures_model =  
                  new Function(*state, Return( STATE(0),
                                              STATE(1)+STATE(2)));
            ROS_DEBUG_STREAM("set measures model end");
            return measures_model;
        }
};

class IMUDynamicalModel: public DynamicalModel
{
    public:
        static const unsigned int state_size       = 3*3+1; // state_size
        static const unsigned int control_size     = 2*3;   // control_size 
        static const unsigned int measures_size    = 3;     // measures_size

        double guz[3]={0.,0., -9.81};
	    Vector guz_;

    public:
        IMUDynamicalModel(  const double dt              = NaN,  // dt
                            const Method method          = HEUN,  // method       
                            const double precision       = 1e-4, // precision
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
                            precision),
            guz_(3, guz)
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

            h_ = 0.1; // initial timestep for dynibex simulations
            adaptative_timestep_ = false;
        }

    protected:
        Function* getDynamicalModel(const IntervalVector* control, Variable* state)
        {
            ROS_DEBUG_STREAM("set IVP dynamical model begin");

            auto dynamical_model =  
                new Function(*state, 
                    ReturnIMU( 0.5*(-STATE(1)*CONTROL(0)-STATE(2)*CONTROL(1)-STATE(3)*CONTROL(2)),
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

            ROS_DEBUG_STREAM("set IVP dynamical model end");
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

#endif
