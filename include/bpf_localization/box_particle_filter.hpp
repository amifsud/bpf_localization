/*** FIXME 
 *
 *  - implement maximum likelihood resqmpling direction choice
 *  - particles are composed of AND/OR(IntervalVector, kernel, maps, ...) and weights
 *  - stop keeping memory of Particles (heavy, to useful)
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

/*** Available algos ***/
// Init
//  * uniformly choosen : 0
//  * uniform           : 1
// Resampling
//  * multinomial       : 0
//  * guaranted         : 1
// Resampling direction
//  * random            : 0
//  * geometrical       : 1
//  * maximum likelihood: 2

/*** Defaults algos versions ***/
#ifndef INIT_METHOD
    #define INIT_METHOD          0
#endif

#ifndef RESAMPLING_METHOD
    #define RESAMPLING_METHOD    0
#endif

#ifndef RESAMPLING_DIRECTION
    #define RESAMPLING_DIRECTION 0
#endif

/*** Available particles subdivision ***/
// * random
// * all dimensions
// * given dimensions

/*** Algos dependencies ***/
#if INIT_METHOD == 1
    #define SUBDIVISE_OVER_ALL_DIMENSIONS
#endif

#if RESAMPLING_DIRECTION == 1 | RESAMPLING_DIRECTION == 2
    #define SUBDIVISE_OVER_GIVEN_DIRECTION
#endif

#include <omp.h>
#include "bpf_localization/dynamical_systems.hpp"

using namespace ibex;

enum BOXES_TYPE{PREDICTION, CORRECTION, RESAMPLING, DEFAULT};

/*! \enum SUBDIVISION_TYPE */
enum SUBDIVISION_TYPE
{
    GIVEN,          /*!< use Particle::subdiviseOverGivenDirection() method */
    ALL_DIMENSIONS, /*!< use Particle::subdiviseOverAllDimensions() method */
    RANDOM          /*!< use Particle::subdiviseOverRandomDimensions() method */
};

/*!
 * \class Particle
 * \brief Particle for the box particle filter 
 *
 * A particle inherit of an interval vector and has a #weight_. 
 * It contains several methods to subdivise it, generating Particles object.
 */
class Particle : public IntervalVector
{
    protected:
        /*! weight of the particle */
        double weight_;

        /*! uniform distribution used to randomly subdivise particles */
        std::uniform_real_distribution<double> uniform_distribution_;

    protected:
        /*** Boxes processing ***/

        #ifdef SUBDIVISE_OVER_ALL_DIMENSIONS
        /*! \fn std::deque<Particle> subdiviseOverAllDimensions(unsigned int dim = 0)
         *
         *  \brief Recusively subdivise over all dimensions of the particle, begining by dim
         *
         *  \param dim dimension we start subdivising
         *  \return list of particles
         * */
        std::deque<Particle> subdiviseOverAllDimensions(unsigned int dim = 0)
        {
            ROS_DEBUG("Subdivise over all dimensions begin");
            std::deque<Particle> boxes, boxes_tmp;

            std::pair<IntervalVector, IntervalVector> pair = this->bisect(dim);

            if(dim < this->size() - 1)
            {
                boxes = Particle(std::get<0>(pair), this->weight_/2.)
                    .subdiviseOverAllDimensions(dim+1);
                boxes_tmp = Particle(std::get<1>(pair), this->weight_/2.)
                    .subdiviseOverAllDimensions(dim+1);;
                boxes.insert(boxes.end(), boxes_tmp.begin(), boxes_tmp.end()); 
            }
            else
            {
                boxes.push_back(Particle(std::get<0>(pair), this->weight_/2.));
                boxes.push_back(Particle(std::get<1>(pair), this->weight_/2.));
            }
            ROS_DEBUG("Subdivise over all dimensions end");
            return boxes;
        }
        #endif

        #ifdef SUBDIVISE_OVER_GIVEN_DIRECTION
        /*! \fn std::deque<Particle> subdiviseOverGivenDirection(const unsigned int dim, const unsigned int N = 1)
         *
         *  \brief Subdivise the dim dimension of the particle (interval vector) N times
         *
         *  \param dim dimension to subdivise
         *  \param N number of subdivisions to make
         *
         *  \return list of particles
         * */
        std::deque<Particle> subdiviseOverGivenDirection
            (const unsigned int dim, const unsigned int N = 1)
        {
            ROS_DEBUG("Subdivise over given dimension begin");
            std::deque<Particle> boxes;
            if(N > 0) boxes.push_back(*this);

            for(unsigned int i = 0; i < N-1; ++i)
            {
                std::pair<IntervalVector, IntervalVector> pair
                    = boxes[0].bisect(dim, 1.-1./(N-i));
                boxes.erase(boxes.begin());
                boxes.push_front(Particle(std::get<0>(pair), 1.));
                boxes.push_back(Particle(std::get<1>(pair), this->weight_/N));
            }

            ROS_DEBUG("Subdivise over given dimension end");
            return boxes;
        }
        #endif

        /*! \fn std::deque<Particle> subdiviseOverRandomDimensions(unsigned int N = 1)
         *
         *  \brief Subdivise over random dimensions of the particle (interval vector) N times
         *
         *  \param N number of subdivisions to make
         *
         *  \return list of particles
         * */
        std::deque<Particle> subdiviseOverRandomDimensions(unsigned int N = 1)
        {
            ROS_DEBUG("Subdivise over random dimension begin");
            std::deque<Particle> boxes;
            if(N > 0) boxes.push_back(*this);
            std::random_device rd;
            std::default_random_engine generator(rd());

            unsigned int direction;

            while (boxes.size() < N)
            {
                direction = int(uniform_distribution_(generator) * this->size());
                std::pair<IntervalVector, IntervalVector> pair
                    = boxes[0].bisect(direction, 0.5);
                boxes.push_back(Particle(std::get<0>(pair), boxes[0].weight_/2.));
                boxes.push_back(Particle(std::get<1>(pair), boxes[0].weight_/2.));
                boxes.pop_front();
            }

            ROS_DEBUG("Subdivise over random dimension end");
            return boxes;
        }

    public:
        /*! \fn Particle(const IntervalVector& box, const double weight) 
         *
         *  \brief Particle constructor
         *
         *  \param box interval vector which is the particle
         *  \param weight weight of the particle
         *
         * */
        Particle(const IntervalVector& box, const double weight): 
                IntervalVector(box),
                weight_(weight),
                uniform_distribution_(0.0,1.0)
        {
            for(auto i = 0; i < box.size(); i++)
            {
                ROS_ASSERT_MSG(box[i].diam() != 0., 
                        "One of the initial box diameter is null");
            }
        }

        /*! \fn std::deque<Particle> subdivise
            (SUBDIVISION_TYPE sub_type = SUBDIVISION_TYPE::RANDOM,
             unsigned int N = 1, unsigned int dim = 0) 
        *   
        *   \brief Generic subdivise method, where we can select the subdivision method
        *
        *   \param sub_type subdivision method defined in #SUBDIVISION_TYPE enum
        *   \param N (usefullness depending on the choosed method) number of subdivisions
        *   \param dim (usefullness depending on the choosed method) dimension to subdivise 
        *
        */
        std::deque<Particle> subdivise
            (SUBDIVISION_TYPE sub_type = SUBDIVISION_TYPE::RANDOM,
             unsigned int N = 1, unsigned int dim = 0)
        {
            std::deque<Particle> particles;

            switch(sub_type)
            {
                case SUBDIVISION_TYPE::RANDOM:
                    particles = subdiviseOverRandomDimensions(N);
                    break;
                case SUBDIVISION_TYPE::ALL_DIMENSIONS:
                    #ifdef SUBDIVISE_OVER_ALL_DIMENSIONS
                    particles = subdiviseOverAllDimensions();
                    #else
                    ROS_ERROR_STREAM("Not compiled method");
                    #endif
                    break;
                case SUBDIVISION_TYPE::GIVEN:
                    #ifdef SUBDIVISE_OVER_GIVEN_DIRECTION
                    particles = subdiviseOverGivenDirection(dim, N);
                    #else
                    ROS_ERROR_STREAM("Not compiled method");
                    #endif
                    break;
                default:
                    ROS_ASSERT("Wrong subdivision type");
            }

            return particles;
        }

        /*! \fn double& weight() 
         *
         *  \return weight of the particle
         *
         * */
        double& weight() { return weight_; }
};

/*! \class Particles
 *
 *  \brief List of Particle
 *
 *  Particles inherit of a std::vector of Particle objects.
 *
 */
class Particles: public std::deque<Particle>
{
    protected:
        /*! float sumOfWeights() 
         *
         *  \return sum of the weights of the Particle objects in the list
         *
         * */
        float sumOfWeights()
        {
            float sum = 0;
            for(auto it= this->begin(); it != this->end(); it++)
                sum += it->weight();
            return sum;
        }

    public:
        /*! Particles(std::deque<Particle> particles) 
         *
         *  \brief Constructor from existing particles
         *
         *  \param particles init particles in a std::deque of Particle objects
         *
         * */
        Particles(std::deque<Particle> particles): std::deque<Particle>(particles)
        {
        }

        /*! Particle() 
         *
         *  \brief Constructor for empty list of particles
         *
         * */
        Particles(): std::deque<Particle>()
        {
        }

        /*** Weight processing **/

        /*! resetWeightsUniformly() 
         *
         *  \brief Reset the weights of the Particle objects to a constant value so that the 
         *         sum over the list is one
         *
         * */
        void resetWeightsUniformly()
        {
            for(auto it = this->begin(); it != this->end(); it++)
                it->weight() = 1./this->size();
        }

        /*! std::deque<float> getCumulativeWeights() 
         *
         *  \brief Get cumulative weights as described in \cite merlinge2018thesis page 19 
         *
         *  \return cumulative weights
         *
         * */
        std::vector<float> getCumulativeWeights()
        {
            std::vector<float> cumulated_weights;
            cumulated_weights.push_back(this->begin()->weight());
            unsigned int i = 0;
            for(auto it= this->begin()+1; it != this->end(); it++, ++i)
                cumulated_weights.push_back(cumulated_weights[i]  
                                            + it->weight());
            return cumulated_weights;
        }

        /*! weigthsNormalization() 
         *
         *  \brief Normalize the weights of the Particle objects so that their sum is one
         *
         * */
        void weigthsNormalization()
        {
            float sum_of_weights = sumOfWeights();
            for(auto it = this->begin(); it != this->end(); it++)
                it->weight() /= sum_of_weights;
        }

        /*** Boxes processing **/

        /*! void subdivise( unsigned int i = 0, 
                        SUBDIVISION_TYPE sub_type = SUBDIVISION_TYPE::RANDOM,
                        unsigned int N = 1, unsigned int dim = 0) 
        *
        *   \brief Subdivise the ith element of the list and delete it
        *
        *   \param sub_type #SUBDIVISION_TYPE
        *   \param N (usefullness determined by the #SUBDIVISION_TYPE) number of subdivisions
        *   \param dim (usefullness determined by the #SUBDIVISION_TYPE) dimension of subdivision 
        *
        */
        void subdivise( unsigned int i = 0, 
                        SUBDIVISION_TYPE sub_type = SUBDIVISION_TYPE::RANDOM,
                        unsigned int N = 1, unsigned int dim = 0)
        {
            this->append(this->operator[](i).subdivise(sub_type, N, dim));
            this->erase (this->begin()+i);
        }

        /*** Appending ***/

        /*! void append(std::deque<Particle> particles) 
         *
         *  \brief Append a list of particles
         *
         *  \param particles std::deque of Particle objects to append
         *
         * */
        void append(std::deque<Particle> particles)
        {
            #pragma omp critical
            this->insert(this->end(), particles.begin(), particles.end());
        }

        /*! void append(Particle particles) 
         *
         *  \brief Append a particle
         *
         *  \param particle Particle to append
         *
         * */
        void append(Particle particle)
        {
            #pragma omp critical
            this->insert(this->end(), particle);
        }
};


/*! \class BoxParticleFilter
 *
 *  \brief Box particle filter, mainly implemented from \cite merlinge2018thesis
 *
 *  This a base class which has to be specialized for your system. 
 *  This class only use the DynamicalModel base class from which particular systems inherit.
 *
 * */
class BoxParticleFilter
{
    protected:
        /*! N maximum number of particles of the box particle filter */
        unsigned int N_;

        /*! uniform distribution used to randomly subdivise particles */
        std::uniform_real_distribution<double> uniform_distribution_;

        /*! dynamical model base class from which particular systems inherit */
        std::shared_ptr<DynamicalModel> dynamical_model_;

        /*! Particles of the box particle filter */
        Particles particles_;

        Particles predicted_particles_;
        Particles corrected_particles_;
        Particles resampled_particles_;

        /*! Is the particles been resampled or not */
        bool resampled_;

        /*! Do we parallelize prediction or not 
         *
         *  **WARNING : DynIbex doesn't seem thread safe, so parallelization don't work**
         *
         * */
        bool parallelize_;

    protected:
        /*! \name Initial paving */

        ///@{
        #if INIT_METHOD == 0 | DOXYGEN
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

        #if INIT_METHOD == 1 | DOXYGEN
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
         *  \param initial_box initial interval vector to subpave
         *
         * */
        void initializeParticles(const IntervalVector& initial_box)
        {
            Particles* particles = getParticlesPtr();
            particles->clear();

            #if INIT_METHOD == 0
            particles->append(uniformSubpaving(Particle(initial_box, 1.), N_));
            #endif
            #if INIT_METHOD == 1
            particles->append(allDimensionsSubpaving(Particle(initial_box, 1.), N_));
            #endif

            particles->resetWeightsUniformly();
        }
        ///@}

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

        /*! \name Number of subdivision choice */
        ///@{

        #if RESAMPLING_METHOD == 0 | DOXYGEN
        /*!  std::vector<unsigned int> multinomialSubdivisions(Particles* particles)
         *
         *  \brief Multinomial algorithm to determine the number of Particle subdivision
         *
         *  Used if RESAMPLING_METHOD == 0
         *
         *  For each particle, determine the number of subdivision to perform, using the multinomial 
         *  algortihm in \cite merlinge2018thesis (algorithm 3 page 19)
         *
         * */
        std::vector<unsigned int> multinomialSubdivisions(Particles* particles)
        {
            ROS_DEBUG_STREAM("Begin multinomial resampling");

            std::vector<float> cumulative_weights = particles->getCumulativeWeights();

            double ui;
            unsigned int j;
            std::vector<unsigned int> n(cumulative_weights.size(), 0.0);
            for(unsigned int i = 0; i < N_; ++i)
            {
                std::default_random_engine generator;
                ui = uniform_distribution_(generator);
                j=0;
                while(ui >= cumulative_weights[j]) j++;
                n[j] += 1;
            }

            ROS_DEBUG_STREAM("End multinomial resampling");
            return n;
        }
        #endif

        #if RESAMPLING_METHOD == 1 | DOXYGEN
        /*!  std::vector<unsigned int> gurantedSubdivisions(Particles* particles)
         *
         *  \brief Guaranted algorithm to determine the number of Particle subdivision
         *
         *  Used if RESAMPLING_METHOD == 1
         *
         *  For each particle, determine the number of subdivision to perform, using the guaranted 
         *  algortihm in \cite merlinge2018thesis (algorithm 6 page 72)
         *
         * */
        std::vector<unsigned int> guarantedSubdivisions(Particles* particles)
        {
            ROS_DEBUG_STREAM("Begin guaranted resampling");

            std::vector<float> cumulative_weights = particles->getCumulativeWeights();

            std::vector<unsigned int> n(cumulative_weights.size(), 1.0);
            unsigned int M = 0;
            for(unsigned int i = 0; n.size(); ++i)
            {
                if(particles->operator[](i).weight() == 0.0) n[i] = 0.0;
                M++;
            }

            double ui;
            unsigned int j;
            std::random_device rd;
            std::default_random_engine generator(rd());
            for(unsigned int i = 0; i < M; ++i)
            {
                ui = uniform_distribution_(generator);
                j=0;
                while(ui >= cumulative_weights[j]) j++;
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
        std::vector<unsigned int> chooseSubdivisions(Particles* particles)
        {
            #if RESAMPLING_METHOD == 0
            return multinomialSubdivisions(particles);
            #elif RESAMPLING_METHOD == 1
            return guarantedSubdivisions(particles);
            #endif
        }
        ///@}

        /*! \name Resampling direction */

        ///@{
        #if RESAMPLING_DIRECTION == 0 | DOXYGEN
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
            std::default_random_engine generator;
            ROS_DEBUG_STREAM("Get random direction for resampling end");
            return int(uniform_distribution_(generator) * box.size());
        }
        #endif

        #if RESAMPLING_DIRECTION == 1 | DOXYGEN
        /*! unsigned int getGeometricalDirection(IntervalVector& box)
         * 
         *  \brief Get geometrical direction for subdivision
         *
         *  Used if RESAMPLING_DIRECTION == 1
         *
         *  The direction is choosed geometrically like in \cite merlinge2018thesis (section 4.3.1 page 87)
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
                norm = diam.norm();// or std::get<2>(it)
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

        #if RESAMPLING_DIRECTION == 2 | DOXYGEN
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

        Particles* getParticlesPtr(BOXES_TYPE boxes_type = BOXES_TYPE::DEFAULT)
        {
            switch(boxes_type)
            {
                case DEFAULT:    return &particles_;           break;
                case PREDICTION: return &predicted_particles_; break;
                case CORRECTION: return &corrected_particles_; break;
                case RESAMPLING: return &resampled_particles_; break;
                default: ASSERT("Wrong specified boxes type");
            }
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
        *   \param parallelize use parallelization or not (**WARNING : DynIbex is not thread safe**)
        *
        */
        BoxParticleFilter(  unsigned int N, 
                            IntervalVector& initial_box,
                            std::shared_ptr<DynamicalModel> dynamical_model,
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
        *   \param parallelize use parallelization or not (**WARNING : DynIbex is not thread safe**)
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
        *   \param parallelize use parallelization or not (**WARNING : DynIbex is not thread safe**)
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
        ///@}

        /*! \name Box particle filter workflow */

        ///@{
        /*! void prediction(IntervalVector& control, BOXES_TYPE input_boxes_type = BOXES_TYPE::DEFAULT)
         *
         *  \brief Predict the Particles evolution using the control
         *
         *  \param control control to apply to each Particle in #particles_
         *
         *  Use the #dynamical_model_ to predict the evolution of each Particle in #particles_ when we 
         *  apply the control
         *
         */
        void prediction(IntervalVector& control, BOXES_TYPE input_boxes_type = BOXES_TYPE::DEFAULT)
        {
            ROS_DEBUG_STREAM("prediction begin");
            assertReady();

            Particles* particles = getParticlesPtr(input_boxes_type);
            predicted_particles_.clear();

            double start, end;
            start = omp_get_wtime();
            #pragma omp parallel for if(parallelize_)
            for(auto it = particles->begin(); it < particles->end(); it++)
            {
                {
                //ROS_INFO_STREAM("Particle in thread : " << omp_get_thread_num());
                predicted_particles_.append(
                      Particle(
                            dynamical_model_->applyDynamics(*it, control),
                            it->weight()));
                }
            }
            end = omp_get_wtime();
            ROS_INFO_STREAM("prediction duration = " << (end - start)); 

            ROS_DEBUG_STREAM("prediction end");
        }

        /*! void correction(const IntervalVector& measures, BOXES_TYPE input_boxes_type = BOXES_TYPE::PREDICTION) 
         *
         *  \brief Correct the prediction by contracting with the measures
         *
         *  \param measures measures used to contract
         *
         *  Use the #dynamical_model_ to link the predicted #particles_ to the measures and contract using the 
         *  contract() method to correct them.
         *
         * */
        void correction(const IntervalVector& measures, BOXES_TYPE input_boxes_type = BOXES_TYPE::PREDICTION)
        {   
            ROS_DEBUG_STREAM("Begin correction");
            assertReady();

            Particles* particles = getParticlesPtr(input_boxes_type);
            corrected_particles_.clear();

            #pragma omp parallel for
            for(auto it = particles->begin(); it < particles->end(); it++)
            {
                IntervalVector predicted_measures  = dynamical_model_->applyMeasures(*it);
                IntervalVector innovation          = predicted_measures & measures;

                if(innovation.volume() > 0)
                {
                    corrected_particles_.append(
                            Particle(contract(innovation, *it),
                                     it->weight() * (innovation.volume()
                                        /predicted_measures.volume()))); 
                }
            }

            // Resampling (if necessary)
            //resampling(BOXES_TYPE::CORRECTION);

            ROS_DEBUG_STREAM("End correction");
        }

        /*! void resampling(BOXES_TYPE input_boxes_type = BOXES_TYPE::CORRECTION)
         *
         *  \brief Resampling the Particles in particles_ if necessary
         *
         *  Use the preprocessor definitions RESAMPLING_METHOD and RESAMPLING_DIRECTION 
         *  to choose the resampling scheme
         *
         */
        void resampling(BOXES_TYPE input_boxes_type = BOXES_TYPE::CORRECTION)
        {
            ROS_DEBUG_STREAM("Will we resample");
            Particles* particles = getParticlesPtr(input_boxes_type);
            resampled_particles_.clear();

            // Check that we need to resample
            double Neff = 0;
            for(unsigned int i = 0; i < particles->size(); ++i)
                Neff += 1./pow((*particles)[i].weight(), 2);
            if(Neff <= 0.7 * particles->size())
            {
                ROS_DEBUG_STREAM("We will resample");

                // Compute number of subdivisions per boxes
                std::vector<unsigned int> n
                    = chooseSubdivisions(particles);

                // Subdivise boxes with ni boxes (delete box if ni=0) 
                unsigned int i = 0;
                unsigned int dir;
                for(auto it = particles->begin(); it < particles->end(); it++, i++)
                {
                    dir = getDirection(*it);
                    resampled_particles_
                        .append(it->subdivise(SUBDIVISION_TYPE::GIVEN, n[i], dir));
                }

                resampled_particles_.weigthsNormalization();
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
        std::shared_ptr<DynamicalModel> dynamicalModel() const { return dynamical_model_; }
        /*! const Particles& getParticles(BOXES_TYPE boxes_type = BOXES_TYPE::DEFAULT) const */
        const Particles& getParticles(BOXES_TYPE boxes_type = BOXES_TYPE::DEFAULT) const
        {
            switch(boxes_type)
            {
                case DEFAULT:    return particles_;           break;
                case PREDICTION: return predicted_particles_; break;
                case CORRECTION: return corrected_particles_; break;
                case RESAMPLING: return resampled_particles_; break;
                default: ASSERT("Wrong specified boxes type");
            }
        }

        ///@}
};

#endif
