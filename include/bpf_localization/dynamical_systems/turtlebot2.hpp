
/*
 * \file   turtlebot2.hpp
 * \brief  Turtle Bot
 * \author Alexis Mifsud
 * \date   2022 January
 */

#define INTEGRATION_METHOD 1

#include "bpf_localization/dynamical_systems/dynamical_systems.hpp"

/** Turtlebot 2 **/

namespace dynamical_systems
{
    /*! \class TurtleBot
     *
     *  \brief Specialization of DynamicalSystem that implement a Turtle Bot
     *
     */
    class TurtleBot: public dynamical_systems::DynamicalSystem
    {
        public:
            TurtleBot(const double dt = 0.01):
                dynamical_systems::DynamicalSystem(state_size, control_size, measures_size, dt)
            {
                #if INTEGRATION_METHOD == 0
                configureGuarantedIntegration(RK4, 1e-6, false, 0.1);  
                #endif

                #if RESAMPLING_DIRECTION == 1
                normalization_values_.push_back(std::make_tuple(0, 0, 1.));
                normalization_values_.push_back(std::make_tuple(1, 1, 1.));
                normalization_values_.push_back(std::make_tuple(2, 2, 1.));
                #endif
            }

        public:
            /*! \name Static const versions of sizes
             *
             *  Attributes which can be used without DoubleIntegrator instanciation 
             *
             */
            ///@{
            /*! State size */
            static const unsigned int state_size    = 3;
            /*! Control size */
            static const unsigned int control_size  = 2;
            /*! Measures size */
            static const unsigned int measures_size = 2;
            ///@}

            /*! \name TurtleBot geometry 
             *
             *  The Turtlebot geometric is constant and public, so static const
             *
             * */
            ///@{
            /*! Wheels radius */
            static constexpr double wheels_radius   = 3.5e-2;
            /*! Distance between wheels */
            static constexpr double wheels_distance = 23e-2;
            ///@}
     
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
