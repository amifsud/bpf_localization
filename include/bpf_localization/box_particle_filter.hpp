/*** FIXME 
 *
 *  - implement maximum likelihood resqmpling direction choice
 *
 */

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

#include "bpf_localization/dynamical_systems.hpp"
#include "ibex/ibex.h"
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <numeric>
#include <random>

#ifndef BOX_PARTICLE_FILTER
#define BOX_PARTICLE_FILTER

using namespace ibex;

enum BOXES_TYPE{PREDICTION, CORRECTION, RESAMPLING, DEFAULT};
enum SUBDIVISION_TYPE{GIVEN, ALL_DIMENSIONS, RANDOM};

class Kernel
{
};

class Particle
{
    protected:
        // Random
        std::uniform_real_distribution<double> uniform_distribution_;

        // For subdivisions
        std::deque<Particle> boxes, boxes_tmp;
        std::pair<IntervalVector, IntervalVector> pair;
        unsigned int direction;

        IntervalVector  box_;
        Kernel          kernel_;
        double          weight_;

    protected:
        /*** Boxes processing ***/

        #ifdef SUBDIVISE_OVER_ALL_DIMENSIONS
        std::deque<Particle> subdiviseOverAllDimensions(unsigned int dim = 0)
        {
            ROS_DEBUG("Subdivise over all dimensions begin");
            boxes.clear();
            boxes_tmp.clear();

            pair = this->box_.bisect(dim); 

            if(dim < this->box_.size() - 1)
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
        std::deque<Particle> subdiviseOverGivenDirection
            (const unsigned int dim, const unsigned int N = 1)
        {
            ROS_DEBUG("Subdivise over given dimension begin");
            boxes.clear();
            if(N > 0) boxes.push_back(*this);

            for(unsigned int i = 0; i < N-1; ++i)
            {
                pair = boxes[0].box_.bisect(dim, 1.-1./(N-i)); 
                boxes.erase(boxes.begin());
                boxes.push_front(Particle(std::get<0>(pair), 1.));
                boxes.push_back(Particle(std::get<1>(pair), this->weight_/N));
            }

            ROS_DEBUG("Subdivise over given dimension end");
            return boxes;
        }
        #endif

        std::deque<Particle> subdiviseOverRandomDimensions(unsigned int N = 1)
        {
            ROS_DEBUG("Subdivise over random dimension begin");
            boxes.clear();
            if(N > 0) boxes.push_back(*this);
            std::random_device rd;
            std::default_random_engine generator(rd());

            while (boxes.size() < N)
            {
                direction = int(uniform_distribution_(generator) * this->box_.size());
                pair = boxes[0].box_.bisect(direction, 0.5); 
                boxes.push_back(Particle(std::get<0>(pair), boxes[0].weight_/2.));
                boxes.push_back(Particle(std::get<1>(pair), boxes[0].weight_/2.));
                boxes.pop_front();
            }

            ROS_DEBUG("Subdivise over random dimension end");
            return boxes;
        }

    public:
        Particle(const IntervalVector& box, const double weight): 
                box_(box),
                pair(std::pair<IntervalVector, IntervalVector>(box, box)),
                weight_(weight),
                uniform_distribution_(0.0,1.0)
        {
            ROS_ASSERT_MSG(box[0].diam() != 0 && box[1].diam() != 0 && box[2].diam() != 0,
                "One of the initial box diameter is null");
            pair = std::pair<IntervalVector, IntervalVector>(box.bisect(0));
        }

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

        const IntervalVector& box() const { return box_; }
        const Kernel& kernel() const { return kernel_; }
        double& weight() { return weight_; }
};

class Particles: public std::deque<Particle>
{
    protected:
        /*** Weights processing ***/

        float sumOfWeights()
        {
            float sum = 0;
            for(auto it= this->begin(); it != this->end(); it++)
                sum += it->weight();
            return sum;
        }

    public:
        /*** Constructors ***/

        Particles(std::deque<Particle> particles): 
            std::deque<Particle>(particles)
        {
        }

        Particles(): 
            std::deque<Particle>()
        {
        }

        /*** Weight processing **/

        void resetWeightsUniformly()
        {
            for(auto it = this->begin(); it != this->end(); it++)
                it->weight() = 1./this->size();
        }

        std::vector<float> getCumulatedWeights()
        {
            std::vector<float> cumulated_weights;
            cumulated_weights.push_back(this->begin()->weight());
            unsigned int i = 0;
            for(auto it= this->begin()+1; it != this->end(); it++, ++i)
                cumulated_weights.push_back(cumulated_weights[i]  
                                            + it->weight());
            return cumulated_weights;
        }

        void weigthsNormalization()
        {
            float sum_of_weights = sumOfWeights();
            for(auto it= this->begin(); it != this->end(); it++)
                it->weight() /= sum_of_weights;
        }

        /*** Boxes processing **/

        void subdivise( unsigned int i = 0, 
                        SUBDIVISION_TYPE sub_type = SUBDIVISION_TYPE::RANDOM,
                        unsigned int N = 1, unsigned int dim = 0)
        {
            this->append(this->operator[](i).subdivise(sub_type, N, dim));
            this->erase(this->begin());
        }

        /*** Appending ***/

        void append(std::deque<Particle> particles)
        {
            this->insert(this->end(), particles.begin(), particles.end());
        }

        void append(Particle particle)
        {
            this->insert(this->end(), particle);
        }
};

class BoxParticleFilter
{
    protected:
        unsigned int N_;                                // Number of particles

        //Random
        std::uniform_real_distribution<double> uniform_distribution_;

        // Dynamical model
        Variable state_variable_;
        DynamicalModel* dynamical_model_;

        // Particles
        Particles particles_;
        Particles predicted_particles_;
        Particles corrected_particles_;
        Particles resampled_particles_;

        // Resampling
        bool resampled_;
        #if RESAMPLING_DIRECTION == 1
        std::map<int, std::pair<int, double>> geometrical_subdivision_map;
        #endif

    protected:
        /*** Paving ***/

        #if INIT_METHOD == 0
        void initializeBoxes(IntervalVector initial_box)
        {
            ROS_DEBUG_STREAM("Uniformly choosen paving initialization begin");
            // We choose to subdvise the initial box with equal size boxes (1)

            Particles* particles = getParticlesPtr();
            particles->clear();
            particles->append(Particle(initial_box, 1.));
            particles->subdivise(0, SUBDIVISION_TYPE::RANDOM, N_);
            particles->resetWeightsUniformly();

            ROS_DEBUG_STREAM("Uniformly choosen paving initialization end");
        }
        #endif

        #if INIT_METHOD == 1
        void initializeBoxes(IntervalVector initial_box)
        {
            ROS_DEBUG_STREAM("Uniform paving initialization begin");

            Particles* particles = getParticlesPtr();
            particles->clear();
            particles->append(Particle(initial_box, 1.));

            unsigned int current_particles_nb = 1;

            while(current_particles_nb*pow(2,dynamical_model_->stateSize()) <= N_)
            {
                for(unsigned int box_i = 0; box_i < current_particles_nb; ++box_i)
                    particles->subdivise
                        (0, SUBDIVISION_TYPE::ALL_DIMENSIONS, dynamical_model_->stateSize());
                current_particles_nb = particles->size();
            }

            particles->resetWeightsUniformly();

            ROS_DEBUG_STREAM("Uniform paving initialization end");
        }
        #endif

        /*** Contraction ***/

        IntervalVector contract(IntervalVector innovation, IntervalVector box)
        {
            // Use a contractor to found the box subset that give the innovation 
            // (i.e. (predicted measure)s & (measures)) by the measures dynamics
            return box;
        }

        /*** Resampling ***/

        #if RESAMPLING_METHOD == 0
        std::vector<unsigned int> 
            chooseSubdivisions(Particles* particles)
        {
            ROS_DEBUG_STREAM("Begin multinomial resampling");

            std::vector<float> cumulated_weights = particles->getCumulatedWeights();

            double ui;
            unsigned int j;
            std::vector<unsigned int> n(cumulated_weights.size(), 0.0);
            for(unsigned int i = 0; i < N_; ++i)
            {
                std::default_random_engine generator;
                ui = uniform_distribution_(generator);
                j=0;
                while(ui >= cumulated_weights[j]) j++;
                n[j] += 1;
            }

            ROS_DEBUG_STREAM("End multinomial resampling");
            return n;
        }
        #endif

        #if RESAMPLING_METHOD == 1
        std::vector<unsigned int> 
            chooseSubdivisions(Particles* particles)
        {
            ROS_DEBUG_STREAM("Begin guaranted resampling");

            std::vector<float> cumulated_weights = particles->getCumulatedWeights();

            std::vector<unsigned int> n(cumulated_weights.size(), 1.0);
            unsigned int M = 0;
            for(unsigned int i = 0; n.size(); ++i)
            {
                if(particles->operator[](i).weight_ == 0.0) n[i] = 0.0;
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
                while(ui >= cumulated_weights[j]) j++;
                n[j] += 1;
            }

            ROS_DEBUG_STREAM("End guaranted resampling");
            return n;
        }
        #endif

        #if RESAMPLING_DIRECTION == 0
        // Random
        unsigned int getDirection(IntervalVector box)
        {
            ROS_DEBUG_STREAM("Get random direction for resampling begin");
            std::default_random_engine generator;
            ROS_DEBUG_STREAM("Get random direction for resampling end");
            return int(uniform_distribution_(generator) * box.size());
        }
        #endif

        #if RESAMPLING_DIRECTION == 1
        // Geometrical
        unsigned int getDirection(IntervalVector box)
        {
            ROS_DEBUG_STREAM("Get geometrical direction for resampling begin");
            std::map<int, int>::iterator it = geometrical_subdivision_map.begin();
            double norm;
            Vector diameters(box.size()), diam(1);
            IntervalVector subvect(1);
            unsigned int i = 0;
            while(it != geometrical_subdivision_map.end())
            {
                subvect = box.subvector(it->first, std::get<0>(it->second));
                diam = subvect.diam();
                norm = diam.norm();
                diameters.put(i, (1./norm)*diam);
                it++;
                i += subvect.size();
            }

            unsigned int i_max = 0;
            for(unsigned int i = 0; i < diameters.size(); ++i)
                if(diameters[i] > diameters[i_max]) i_max = i;

            ROS_DEBUG_STREAM("Get geometrical direction for resampling end");
            return i_max; 
        }
        #endif

        #if RESAMPLING_DIRECTION == 2
        // Maximum Likelihood
        unsigned int getDirection(IntervalVector box)
        {
            ROS_DEBUG_STREAM("Get maximum likelihood direction for resampling begin");
            ROS_ERROR_STREAM("Not implemented yet");
            ROS_DEBUG_STREAM("Get maximum likelihood direction for resampling begin");
            return 0;
        }
        #endif

        void resampling(BOXES_TYPE boxes_type = BOXES_TYPE::CORRECTION)
        {
            ROS_DEBUG_STREAM("Will we resample");
            Particles* particles = getParticlesPtr(boxes_type);
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
                for(auto it = particles->begin(); it == particles->end(); it++, i++)
                {
                    dir = getDirection(it->box());
                    resampled_particles_
                        .append(it->subdivise(SUBDIVISION_TYPE::GIVEN, n[i], dir));
                }

                resampled_particles_.weigthsNormalization();
                ROS_DEBUG_STREAM("End resampling");
            }
            else{ ROS_DEBUG_STREAM("We don't resample"); }
        }

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

    public:
        /*** Box particle filter steps ***/

        BoxParticleFilter(  unsigned int N, IntervalVector& initial_box,
                            DynamicalModel* dynamical_model)
            : uniform_distribution_(0.0,1.0), dynamical_model_(dynamical_model)
        {
            ROS_ASSERT_MSG(dynamical_model->stateSize() == initial_box.size(), 
                    "State size and initial box size not consistent");

            N_          = N;
            initializeBoxes(initial_box);
        }

        void prediction(IntervalVector& control, 
                        BOXES_TYPE input_boxes_type = BOXES_TYPE::DEFAULT)
        {
            ROS_DEBUG_STREAM("prediction begin");
            Particles* particles = getParticlesPtr(input_boxes_type);
            predicted_particles_.clear();

            for(auto it = particles->begin(); it < particles->end(); it++)
                predicted_particles_.append(
                        Particle(dynamical_model_->applyDynamics(it->box(), control),
                                 it->weight()));

            ROS_DEBUG_STREAM("prediction end");
        }

        void correction(const IntervalVector& measures,
                        BOXES_TYPE input_boxes_type = BOXES_TYPE::PREDICTION) 
        {   
            ROS_DEBUG_STREAM("Begin correction");
            Particles* particles = getParticlesPtr(BOXES_TYPE::RESAMPLING);
            corrected_particles_.clear();

            IntervalVector predicted_measures = IntervalVector(measures.size());
            IntervalVector innovation         = IntervalVector(measures.size());

            for(auto it = particles->begin(); it < particles->end(); it++)
            {
                predicted_measures  = dynamical_model_->applyMeasures(it->box());
                innovation          = predicted_measures & measures;

                if(innovation.volume() > 0)
                {
                    corrected_particles_.append(
                            Particle(contract(innovation, it->box()),
                                     it->weight() * (innovation.volume()
                                                /predicted_measures.volume()))); 
                }
            }

            // Resampling (if necessary)
            resampling(BOXES_TYPE::CORRECTION);

            ROS_DEBUG_STREAM("End correction");
        }

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

        const unsigned int& N() const { return N_; }
        const DynamicalModel* dynamicalModel() const { return dynamical_model_; }
};

#endif
