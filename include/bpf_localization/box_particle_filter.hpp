/*** FIXME 
 *
 *  - implement maximum likelihood resqmpling direction choice
 *  - particles are composed of AND/OR(IntervalVector, kernel, maps, ...) and weights
 *
 */

/**
 * \file   box_particle_filter.hpp
 * \brief  Box particle filter
 * \author Alexis Mifsud
 * \date   2022 January
 */

#ifndef BOX_PARTICLE_FILTER
#define BOX_PARTICLE_FILTER

/*! \name Default MACROS
 *
 *  \brief Those MACROS are used to define the Box Particle Filter version we use. 
 * 
 *  **Values here are default values**
 *
 * */
///@{
/*! INIT_METHOD
 *
 *  \brief Initialization method
 *
 *  * 0 : Subpaving defined by choosing directions of bisections randomly with 
 *          an uniform distribution
 *  * 1 : Subpaving defined by bisecting over all directions
 */
#ifndef INIT_METHOD
    #define INIT_METHOD          0
#endif

/*! RESAMPLING_METHOD
 *
 *  \brief Resampling method
 *
 *  * 0 : Multinomial resampling : For each particle, determine the number of subdivision 
 *          to perform, 
 *        using the multinomial algortihm in \cite merlinge2018thesis (algorithm 3 page 19)
 *  * 1 : Guaranted resampling : For each particle, determine the number of subdivision to 
 *          perform, using the guaranted  algortihm in \cite merlinge2018thesis 
 *          (algorithm 6 page 72)
 *
 */
#ifndef RESAMPLING_METHOD
    #define RESAMPLING_METHOD    0
#endif

/*! RESAMPLING_DIRECTION
 *
 * \brief Resamplinmg direction
 *
 * * 0 : Random direction : The direction is choosed randomly with an uniform distribution
 * * 1 : Geometrical direction : The direction is choosed geometrically like in 
 *          \cite merlinge2018thesis (section 4.3.1 page 87)
 * * 2 : Maximum likelihood : The direction is choosed maximizing the likelihood like in 
 *       \cite merlinge2018thesis (section 4.3.2 page 88)
 *
 */
#ifndef RESAMPLING_DIRECTION
    #define RESAMPLING_DIRECTION 0
#endif
///@}

#include <omp.h>
#include "bpf_localization/particles.hpp"
#include "bpf_localization/dynamical_systems/dynamical_systems.hpp"

using namespace ibex;

/*! Contains everithing related to the box particle filter*/
namespace bpf
{
    /*! \class BoxParticleFilter
     *
     *  \brief Box particle filter, mainly implemented from \cite merlinge2018thesis
     *
     *  This a base class which has to be specialized for your system. 
     *  This class only use the DynamicalModel base class from which particular systems 
     *  inherit.
     *
     * */
    class BoxParticleFilter
    {
        public:
            /*! \name Constructors */

            ///@{
            /*! BoxParticleFilter(  unsigned int N, 
                                IntervalVector& initial_box,
                                std::shared_ptr<DynamicalModel> dynamical_model,
                                bool parallelize = false)
            *
            *   \brief Constructor
            *
            *   \param N number of particles
            *   \param initial_box initial interval vector that encapsulate all possible states
            *   \param dynamical dynamical model on which to apply the filter
            *   \param parallelize use parallelization or not 
            *           (**WARNING : DynIbex is not thread safe**)
            *
            */
            BoxParticleFilter(  unsigned int N, 
                                IntervalVector& initial_box,
                                std::shared_ptr<dynamical_systems::DynamicalSystem> dynamical_model,
                                bool parallelize = false)
                : uniform_distribution_(0.0, 1.0)
            {
                ROS_ASSERT_MSG(dynamical_model->stateSize() == initial_box.size(), 
                        "State size and initial box size not consistent");
     
                dynamical_model_ = dynamical_model;            
                N_ = N;
                initializeParticles(initial_box);
                parallelize_ = parallelize;
            }

            /*! BoxParticleFilter(  unsigned int N, 
                                IntervalVector& initial_box,
                                bool parallelize = false)
            *
            *   \brief Constructor 
            *
            *   Constructor without dynamical model (to be set after, assertReady() will prevent 
            *   using the filter without it)
            *
            *   \param N number of particles
            *   \param initial_box initial interval vector that encapsulate all possible states
            *   \param parallelize use parallelization or not 
            *           (**WARNING : DynIbex is not thread safe**)
            *
            */
            BoxParticleFilter(  unsigned int N, 
                                const IntervalVector& initial_box,
                                bool parallelize = false)
                : uniform_distribution_(0.0, 1.0)
            {
                N_ = N;
                initializeParticles(initial_box);
                parallelize_ = parallelize;
            }

            /*! BoxParticleFilter(  unsigned int N, 
                                const Particles& particles,
                                bool parallelize = false)
            *
            *   \brief Constructor 
            *
            *   Constructor without dynamical model (to be set after, assertReady() will prevent 
            *   using the filter without it) and from an existing set of particles
            *
            *   \param N number of particles
            *   \param particles initial set of Particles
            *   \param parallelize use parallelization or not 
            *           (**WARNING : DynIbex is not thread safe**)
            *
            */
            BoxParticleFilter(  unsigned int N, 
                                const Particles& particles,
                                bool parallelize = false)
                : uniform_distribution_(0.0, 1.0)
            {
                N_ = N;
                particles_ = particles;
                parallelize_ = parallelize;
            }

            /*! BoxParticleFilter(  unsigned int N, 
                                    const Particles& particles,
                                    std::shared_ptr<DynamicalModel> dynamical_model,
                                    bool parallelize = false)
            *
            *   \brief Constructor
            *
            *   \param N number of particles
            *   \param particles initial set of Particles
            *   \param dynamical dynamical model on which to apply the filter
            *   \param parallelize use parallelization or not 
            *           (**WARNING : DynIbex is not thread safe**)
            *
            */
            BoxParticleFilter(  unsigned int N, 
                                const Particles& particles,
                                std::shared_ptr<dynamical_systems::DynamicalSystem> dynamical_model,
                                bool parallelize = false)
                : uniform_distribution_(0.0, 1.0)
            {
                dynamical_model_ = dynamical_model;            
                N_ = N;
                particles_ = particles;
                parallelize_ = parallelize;
            }
            ///@}

            /*! \name Box particle filter workflow */

            ///@{
            /*! void prediction(IntervalVector& control)
             *
             *  \brief Predict the Particles evolution using the control
             *
             *  \param control control to apply to each Particle in #particles_
             *
             *  Use the #dynamical_model_ to predict the evolution of each Particle in #particles_ 
             *  when we apply the control
             *
             */
            void prediction(IntervalVector& control)
            {
                ROS_DEBUG_STREAM("prediction begin");
                assertReady();

                double start, end;
                start = omp_get_wtime();
                #pragma omp parallel for if(parallelize_)
                for(auto it = particles_.begin(); it < particles_.end(); it++)
                {
                    {
                        //ROS_INFO_STREAM("Particle in thread : " << omp_get_thread_num());
                        it->updateBox(dynamical_model_->applyDynamics(*it, control));
                    }
                }
                end = omp_get_wtime();
                ROS_INFO_STREAM("prediction duration = " << (end - start)); 

                ROS_DEBUG_STREAM("prediction end");
            }

            /*! void correction(const IntervalVector& measures) 
             *
             *  \brief Correct the prediction by contracting with the measures
             *
             *  \param measures measures used to contract
             *
             *  Use the #dynamical_model_ to link the predicted #particles_ to the measures and 
             *  contract using the contract() method to correct them.
             *
             * */
            void correction(const IntervalVector& measures)
            {   
                ROS_DEBUG_STREAM("Begin correction");
                assertReady();

                #pragma omp parallel for if(parallelize_)
                for(auto it = particles_.begin(); it < particles_.end(); it++)
                {
                    IntervalVector predicted_measures  = dynamical_model_->applyMeasures(*it);
                    IntervalVector innovation          = predicted_measures & measures;

                    if(innovation.volume() > 0)
                    {
                        it->updateBox(contract(innovation, *it));
                        it->updateWeight(it->weight() 
                                            * (innovation.volume()/predicted_measures.volume()));
                    }
                }

                resampling();

                ROS_DEBUG_STREAM("End correction");
            }

            /*! void resampling()
             *
             *  \brief Resampling the Particles in particles_ if necessary
             *
             *  To determine if the resempling is necessary, we use the 
             *  criterion of \cite merlinge2018thesis page 20
             *
             *  Use the preprocessor definitions RESAMPLING_METHOD and RESAMPLING_DIRECTION 
             *  to choose the resampling scheme
             *
             */
            void resampling()
            {
                ROS_DEBUG_STREAM("Will we resample");

                // Check that we need to resample
                double Neff = 0;
                for(unsigned int i = 0; i < particles_.size(); ++i)
                    Neff += 1./pow(particles_[i].weight(), 2);
                if(Neff <= 0.7 * particles_.size())
                {
                    ROS_DEBUG_STREAM("We will resample");

                    // Compute number of subdivisions per boxes
                    std::vector<unsigned int> n = chooseSubdivisions();

                    // Subdivise boxes with ni boxes (delete box if ni=0) 
                    unsigned int i = 0;
                    unsigned int dir;
                    for(auto it = particles_.begin(); it < particles_.end(); it++, i++)
                    {
                        dir = getDirection(*it);
                        particles_.append(it->subdivise(SUBDIVISION_TYPE::GIVEN, n[i], dir));
                        particles_.erase(it);
                    }

                    particles_.weigthsNormalization();
                    ROS_DEBUG_STREAM("End resampling");
                }
                else{ ROS_DEBUG_STREAM("We don't resample"); }
            }
            ///@}

            /*! \name Getters */
            ///@{

            /*! const unsigned int& N() const */
            const unsigned int& N() const { return N_; }
            /*! std::shared_ptr<DynamicalModel> dynamicalModel() const */
            std::shared_ptr<dynamical_systems::DynamicalSystem> 
                dynamicalModel() const { return dynamical_model_; }

            /*! const Particles& getParticles() const */
            const Particles& getParticles() const { return particles_; }

            ///@}

        protected:

            /*! virtual IntervalVector contract(IntervalVector& innovation, IntervalVector& box)
             *
             *  \brief Contract box according to the innovation
             *
             *  \param innovation
             *  \param box box to contract
             *
             *  Use a contractor to found the box subset that give the innovation 
             *  (i.e. (predicted measures) & (measures)) by the measures dynamics
             *
             */
            virtual IntervalVector contract(IntervalVector& innovation, IntervalVector& box)
            {
                ROS_ASSERT_MSG(false, "Contraction with respect to innovation not set");
                return box;
            }

            /*! assertReady()
             *
             *  \brief Assert that the box particle filter can be used
             *
             */
            void assertReady()
            {
                assert(dynamical_model_ != NULL && "Dynamical model not set");
                assert(N_ > 0 && "Number of particles not set (should be in constrctor");
                assert(particles_.size() > 0 && "There is no particles");
            }

            /*! \name Initial paving */

            ///@{
            #if INIT_METHOD == 0 | defined DOXYGEN
            /*! \fn void uniformSubpaving(Particles* particles) 
             *
             *  \brief Particles subpaving from the subdivision of an initial box
             *
             *  Used if INIT_METHOD == 0
             *
             *  **Uniform subpaving method**
             *
             *  \param N exact number of resulting particles
             *  \param particles the Particles object that contains the init Particle
             *
             * */
            Particles uniformSubpaving(Particle particle, unsigned int N)
            {
                ROS_DEBUG_STREAM("Uniformly choosen paving initialization begin");
                Particles particles(particle.subdivise(SUBDIVISION_TYPE::RANDOM, N));
                ROS_DEBUG_STREAM("Uniformly choosen paving initialization end");
                return particles;
            }
            #endif

            #if INIT_METHOD == 1 | defined DOXYGEN
            /*! \fn void allDimensionsSubpaving(Particles* particles) 
             *
             *  \brief Particles subpaving from the subdivision of an initial box
             *
             *  Used if INIT_METHOD == 1
             *
             *  **Subpaving over all dimensions method**
             *  
             *  \param N maximum of resulting particles
             *  \param particles the Particles object that contains the initial Particle objects
             *
             * */
            Particles allDimensionsSubpaving(Particle particle, unsigned int N)
            {
                ROS_DEBUG_STREAM("Uniform paving initialization begin");
                Particles particles(particle);

                unsigned int current_particles_nb = 1;

                while(current_particles_nb*pow(2,dynamical_model_->stateSize()) <= N)
                {
                    for(unsigned int box_i = 0; box_i < current_particles_nb; ++box_i)
                    {
                        particles.append(
                            particles[0].subdivise(SUBDIVISION_TYPE::ALL_DIMENSIONS));
                        particles.erase(particles.begin());
                    }
                    current_particles_nb = particles.size();
                }

                ROS_DEBUG_STREAM("Uniform paving initialization end");

                return particles;
            }
            #endif

            /*! void initializeParticles(const IntervalVector& initial_box)
             *
             *  \brief Initialization of the Particles from an initial interval vector
             *
             *  Depending on INIT_METHOD macro, the initialization method can be choosed
             *
             *  \param initial_box initial interval vector to subpave
             *
             * */
            void initializeParticles(const IntervalVector& initial_box)
            {
                #if INIT_METHOD == 0
                particles_.append(uniformSubpaving(Particle(initial_box, 1.), N_));
                #endif
                #if INIT_METHOD == 1
                particles_.append(allDimensionsSubpaving(Particle(initial_box, 1.), N_));
                #endif

                particles_.resetWeightsUniformly();
            }
            ///@}

            /*! \name Number of subdivision choice */
            ///@{

            #if RESAMPLING_METHOD == 0 | defined DOXYGEN
            /*!  std::vector<unsigned int> multinomialSubdivisions(Particles* particles)
             *
             *  \brief Multinomial algorithm to determine the number of Particle subdivision
             *
             *  Used if RESAMPLING_METHOD == 0
             *
             *  For each particle, determine the number of subdivision to perform, using the 
             *  multinomial algortihm in \cite merlinge2018thesis (algorithm 3 page 19)
             *
             * */
            std::vector<unsigned int> multinomialSubdivisions(std::vector<double> uis)
            {
                ROS_DEBUG_STREAM("Begin multinomial resampling");

                std::vector<float> cumulative_weights = particles_.getCumulativeWeights();

                unsigned int j;
                std::vector<unsigned int> n(cumulative_weights.size(), 0);
                for(unsigned int i = 0; i < N_; ++i)
                {
                    j=0;
                    while(uis[i] - cumulative_weights[j] >= 1e-5) j++;
                    n[j] += 1;
                }

                ROS_DEBUG_STREAM("End multinomial resampling");
                return n;
            }
            #endif

            #if RESAMPLING_METHOD == 1 | defined DOXYGEN
            /*!  std::vector<unsigned int> gurantedSubdivisions(Particles* particles)
             *
             *  \brief Guaranted algorithm to determine the number of Particle subdivision
             *
             *  Used if RESAMPLING_METHOD == 1
             *
             *  For each particle, determine the number of subdivision to perform, using the 
             *  guaranted algortihm in \cite merlinge2018thesis (algorithm 6 page 72)
             *
             * */
            std::vector<unsigned int> guarantedSubdivisions(std::vector<double> uis)
            {
                ROS_DEBUG_STREAM("Begin guaranted resampling");

                std::vector<float> cumulative_weights = particles_.getCumulativeWeights();

                std::vector<unsigned int> n(cumulative_weights.size(), 1.0);
                unsigned int M = 0;
                for(unsigned int i = 0; i < n.size(); ++i)
                {
                    if(particles_.operator[](i).weight() <= 1e-5) 
                    {
                        n[i] = 0.0;
                        M++;
                    }
                }

                unsigned int j;
                for(unsigned int i = 0; i < M; ++i)
                {
                    j=0;
                    while(uis[i] - cumulative_weights[j] >= 1e-7) j++;
                    n[j] += 1;
                }

                ROS_DEBUG_STREAM("End guaranted resampling");
                return n;
            }
            #endif

            /*!  std::vector<unsigned int> gurantedSubdivisions(Particles* particles)
             *
             *  \brief Determine the number of Particle subdivision
             *
             *  For each particle, determine the number of subdivision to perform
             *
             *  \param particles Particles to subdivise
             *
             * */
            std::vector<unsigned int> 
                chooseSubdivisions(std::vector<double> uis_in = std::vector<double>(0))
            {
                std::vector<double> uis;
                if(uis_in.size() == 0)
                {
                    for(auto i = 0; i < N_; i++)
                        uis.push_back(uniform_distribution_.get());
                }
                else
                {
                    uis = uis_in;
                }

                particles_.weigthsNormalization();

                #if RESAMPLING_METHOD == 0
                return multinomialSubdivisions(uis);
                #elif RESAMPLING_METHOD == 1
                return guarantedSubdivisions(uis);
                #endif
            }
            ///@}

            /*! \name Resampling direction */

            ///@{
            #if RESAMPLING_DIRECTION == 0 | defined DOXYGEN
            /*! unsigned int getRandomDirection(IntervalVector& box)
             * 
             *  \brief Get random direction for subdivision
             *
             *  Used if RESAMPLING_DIRECTION == 0
             *
             *  The direction is choosed randomly with an uniform distribution
             *
             *  \param size size of the box to subdivise
             *  \result the dimension to subdivise
             *
             */
            unsigned int getRandomDirection(IntervalVector& box)
            {
                ROS_DEBUG_STREAM("Get random direction for resampling begin");
                unsigned int dir = int(uniform_distribution_.get() * box.size());
                ROS_DEBUG_STREAM("Get random direction for resampling end");
                return dir;
            }
            #endif

            #if RESAMPLING_DIRECTION == 1 | defined DOXYGEN
            /*! unsigned int getGeometricalDirection(IntervalVector& box)
             * 
             *  \brief Get geometrical direction for subdivision
             *
             *  Used if RESAMPLING_DIRECTION == 1
             *
             *  The direction is choosed geometrically like in \cite merlinge2018thesis 
             *  (section 4.3.1 page 87)
             *
             *  \param size size of the box to subdivise
             *  \result the dimension to subdivise
             *
             */
            unsigned int getGeometricalDirection(IntervalVector& box)
            {
                ROS_DEBUG_STREAM("Get geometrical direction for resampling begin");

                double norm;
                Vector diameters(box.size()), diam(1);
                IntervalVector subvect(1);
                unsigned int i = 0;
                for(auto& it : dynamical_model_->normalization_values_)
                {
                    subvect = box.subvector(std::get<0>(it), std::get<1>(it));
                    diam = subvect.diam();
                    norm = std::get<2>(it);//diam.norm();
                    diameters.put(i, (1./norm)*diam);
                    i += subvect.size();
                }

                unsigned int i_max = 0;
                for(unsigned int i = 0; i < diameters.size(); ++i)
                    if(diameters[i] > diameters[i_max]) i_max = i;

                ROS_DEBUG_STREAM("Get geometrical direction for resampling end");
                return i_max; 
            }
            #endif

            #if RESAMPLING_DIRECTION == 2 | defined DOXYGEN
            /*! unsigned int getMaximumlikelihoodDirection(IntervalVector& box)
             * 
             *  \brief Get maximum likelihood direction for subdivision
             *
             *  Used if RESAMPLING_DIRECTION == 2
             *
             *  The direction is choosed maximizing the likelihood like in 
             *  \cite merlinge2018thesis (section 4.3.2 page 88)
             *
             *  \param size size of the box to subdivise
             *  \result the dimension to subdivise
             *
             */
            unsigned int getMaximumLikelihoodDirection(IntervalVector& box)
            {
                ROS_DEBUG_STREAM("Get maximum likelihood direction for resampling begin");
                ROS_ERROR_STREAM("Not implemented yet");
                ROS_DEBUG_STREAM("Get maximum likelihood direction for resampling begin");
                return 0;
            }
            #endif

            /*! unsigned int getDirection(IntervalVector& box)
             * 
             *  \brief Get maximum likelihood direction for subdivision
             *
             *  The direction is choosed maximizing the likelihood like in 
             *  \cite merlinge2018thesis (section 4.3.2 page 88)
             *
             *  \param size size of the box to subdivise
             *  \result the dimension to subdivise
             *
             */
            unsigned int getDirection(IntervalVector& box)
            {
                #if RESAMPLING_DIRECTION == 0
                return getRandomDirection(box);
                #elif RESAMPLING_DIRECTION == 1
                return getGeometricalDirection(box);
                #elif RESAMPLING_DIRECTION == 2
                retrun getMaximumLikelihoodDirection(box);
                #endif
            }
            ///@}

        protected:
            /*! N maximum number of particles of the box particle filter */
            unsigned int N_;

            /*! uniform distribution used to randomly subdivise particles */
            UniformDistribution uniform_distribution_;

            /*! dynamical model base class from which particular systems inherit */
            std::shared_ptr<dynamical_systems::DynamicalSystem> dynamical_model_;

            /*! Particles of the box particle filter */
            Particles particles_;

            /*! Is the particles been resampled or not */
            bool resampled_;

            /*! Do we parallelize prediction or not 
             *
             *  **WARNING : DynIbex doesn't seem thread safe, 
             *  so parallelization don't work**
             *
             * */
            bool parallelize_;
    };

} // namespace bpf
#endif
