
/**
 * \file   particles.hpp
 * \brief  Particles for Box Particle Filter
 * \author Alexis Mifsud
 * \date   2022 January
 */

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

#include "bpf_localization/utils.hpp"

using namespace ibex;

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

        #ifdef SUBDIVISE_OVER_ALL_DIMENSIONS | DOXYGEN
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

        #ifdef SUBDIVISE_OVER_GIVEN_DIRECTION | DOXYGEN
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


