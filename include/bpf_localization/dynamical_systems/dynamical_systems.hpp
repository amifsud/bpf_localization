/** FIXME
 *
 *  - control in state to not reconstruct objects ?
 *  - specific dynamcal models witht there tests and examples are plugins ?
 *  - function composition to define model when we don't use dynibex
**/

/**
 * \file   dynamical_systems.hpp
 * \brief  Dynamical Systems
 * \author Alexis Mifsud
 * \date   2022 January
 */

#ifndef DYNAMICAL_SYSTEMS
#define DYNAMICAL_SYSTEMS

/*! \name Default MACROS
 *
 *  \brief Those MACROS are used to define the dynamical systems integration scheme.
 *          **Values here are default values**
 *
 * */
///@{

/*! INTEGRATION_METHOD
 *
 *  \brief Integration method
 *
 *  * 0 : guaranted with Dynibex IVPs
 *  * 1 : Non guaranted interval integration
 *  * 2 : Non guaranted integration with affine arithmetic
 *
 */
#ifndef INTEGRATION_METHOD
    #define INTEGRATION_METHOD 1
#endif

//@}

#include "bpf_localization/utils.hpp"

using namespace ibex;

/*! \name Simplification MACROS
 *
 *  \brief Those MACROS are used to simplify dynamical models reading and writing
 *
 * */
///@{

#define STATE(x) state->operator[](x)
#define CONTROL(x) control->operator[](x)

///@}

/*! Contains everithing related to the dynamical systems */
namespace dynamical_systems
{
    /*! \class DynamicalModel 
     *
     *  \brief Abstract class that represent dynamical systems
     *
     *  This class is intended to be used by algorithms that are 
     *  not dependant of specificitied of each dynamical systems
     *
     *  Specialize this class to use those algorithms with the corresponding
     *  dynamical systems
     *
     *
     * */
    class DynamicalSystem
    {
        public:
            /*! DynamicalSystem( unsigned int state_size, unsigned int control_size, 
                            unsigned int measures_size, double dt,
                            Vector measures_noise_diams, Vector process_noise_diams)
            *
            *   \brief DynamicalSystem constructor
            *
            *   \param state_size size of the dynamical system state
            *   \param control_size size of the dynamical system control
            *   \param measures_size size of the dynamical system measures
            *   \param dt integration sampling period
            *
            */
            DynamicalSystem( unsigned int state_size, unsigned int control_size, 
                            unsigned int measures_size, double dt)
                :dt_(dt), state_size_(state_size), 
                 control_size_(control_size), measures_size_(measures_size)
            {
                ROS_ASSERT_MSG(state_size > 0, "State size has to be greater than 0");
            }
     
            IntervalVector applyDynamics(const IntervalVector& box, 
                                         const IntervalVector& control)
            {
                //assertReady();
                Variable state(state_size_);
                IntervalVector result(state_size_, Interval(0., 0.));
                Function* dynamical_model = computeDynamicalModel(&control, &state);

                #if INTEGRATION_METHOD == 0
                    ROS_DEBUG_STREAM("guaranted integration method");
                    ivp_ode problem = ivp_ode(*dynamical_model, 0.0, box);

                    simulation simu(&problem, dt_, integration_method_, precision_, h_);
                    simu.run_simulation();

                    if(adaptative_timestep_) 
                        h_ = simu.list_solution_g.back().time_j.diam();
                    result = simu.get_last();
                #elif INTEGRATION_METHOD == 1
                    ROS_DEBUG_STREAM("non guaranted interval HEUN integration method");
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
                    ROS_DEBUG_STREAM("non guaranted affine HEUN integration method");
                    IntervalVector k1 = dynamical_model->eval_vector(box);
                    IntervalVector k2 = dynamical_model->eval_vector(box+dt_*k1);
                    result = box + (0.5*dt_)*(k1+k2);
                #else 
                    ROS_ASSERT(false && "Wrong integration method in preprocessor configuration");
                #endif

                delete dynamical_model;
                return result;
            }

            IntervalVector applyMeasures(const IntervalVector& state)
            {
                //assertReady();
                Variable state_variable(state_size_);
                Function* measures_model = computeMeasuresModel(&state_variable);
                return measures_model->eval_vector(state);
            }

            const double& dt()                 const { return dt_; }
            const unsigned int& stateSize()    const { return state_size_; }
            const unsigned int& controlSize()  const { return control_size_; }
            const unsigned int& measuresSize() const { return measures_size_; }

            #if INTEGRATION_METHOD == 0
            void configureGuarantedIntegration( Method integration_method = RK4, 
                                                double precision = 1e-6, 
                                                bool adaptative_timestep = false, 
                                                double h = 0.1)
            {
                adaptative_timestep_ = adaptative_timestep;
                integration_method_  = integration_method;
                precision_           = precision;
                h_                   = h;
            }
            #endif

        protected:
            virtual Function* computeDynamicalModel
                (const IntervalVector* control, Variable* state) = 0;
            virtual Function* computeMeasuresModel(Variable* state) = 0;

            void assertReady()
            {
                Variable state(state_size_);
                IntervalVector control(state_size_, Interval(0.,0.));

                Function* dynamical_model = computeDynamicalModel( &control, &state);

                assert(dynamical_model != NULL && "dynamical_model_ not set");

                dynamical_model = computeMeasuresModel(&state);
                assert(dynamical_model != NULL  && "measures_model_ not set");

                assert(state_size_ != 0         && "state_size not set");
                assert(control_size_ != 0       && "control_size not set");
                assert(measures_size_ != 0      && "measures_size not set");
            }

        protected:
            double dt_; 

            unsigned int state_size_;
            unsigned int control_size_;
            unsigned int measures_size_;

            #if INTEGRATION_METHOD == 0
            Method integration_method_ = RK4;
            bool adaptative_timestep_  = false;
            double precision_          = 1e-6;
            double h_                  = 0.1;
            #endif

        #if RESAMPLING_DIRECTION == 1
        public:
            std::vector<std::tuple<int, int, double>> normalization_values_;
        #endif

    };

    /** Double integrator **/

    class DoubleIntegrator: public DynamicalSystem
    {
        public:
            static const unsigned int state_size       = 2;         // state_size
            static const unsigned int control_size     = 1;         // control_size 
            static const unsigned int measures_size    = 1;         // measures_size

        public:
            DoubleIntegrator(const double dt = 0.01): 
                DynamicalSystem(state_size, control_size, measures_size, dt)
            {
                #if INTEGRATION_METHOD == 0
                configureGuarantedIntegration(RK4, 1e-6, false, 0.1); 
                #endif
            }

        protected:
            Function* computeDynamicalModel(const IntervalVector* control, Variable* state)
            {
                ROS_DEBUG_STREAM("set dynamical model begin");
                auto dynamical_model =  new Function(*state, Return(STATE(1), CONTROL(0)));
                ROS_DEBUG_STREAM("set dynamical model end");
                return dynamical_model;
            }

            Function* computeMeasuresModel(Variable* state)
            {
                ROS_DEBUG_STREAM("set measures model begin");
                auto measures_model = new Function(*state, STATE(0));
                ROS_DEBUG_STREAM("set measures model end");
                return measures_model;
            }
    };

} // namespace dynamical_systems
#endif
