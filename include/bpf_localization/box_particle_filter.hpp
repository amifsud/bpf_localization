#include "ibex/ibex.h"
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <numeric>
#include <random>

/*** FIXME 
 *
 *  - Every test should test only code in the same class
 *  - Each more genral test should call sub tests ?
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

/*** Algos dependencies ***/
#if INIT_METHOD == 1
    #define SUBDIVISE_OVER_ALL_DIMENSIONS
#endif

#if RESAMPLING_DIRECTION == 1 | RESAMPLING_DIRECTION == 2
    #define SUBDIVISE_OVER_GIVEN_DIRECTION
#endif

using namespace ibex;

enum BOXES_TYPE{PREDICTION, CORRECTION, RESAMPLING, DEFAULT};
enum SUBDIVISION_TYPE{GIVEN, ALL_DIMESIONS, RANDOM};

class Particle
{
    protected:
        // Random
        std::uniform_real_distribution<double> uniform_distribution_;

        // For subdivisions
        std::deque<Particle> boxes, boxes_tmp;
        std::pair<IntervalVector, IntervalVector> pair;
        unsigned int direction;

    public:
        IntervalVector box_;
        float weight_;

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
        Particle(IntervalVector box, const double weight): 
            box_(box),
            weight_(weight),
            pair(box.bisect(0)),
            uniform_distribution_(0.0,1.0)
        {
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
                case SUBDIVISION_TYPE::ALL_DIMESIONS:
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
};

class Particles: public std::deque<Particle>
{
    protected:
        /*** Weights processing ***/

        float sumOfWeights()
        {
            float sum = 0;
            for(auto it= this->begin(); it != this->end(); it++)
                sum += it->weight_;
            return sum;
        }

    public:
        /*** Constructors ***/

        Particles(std::deque<Particle> particles): std::deque<Particle>(particles)
        {
        }

        Particles(): std::deque<Particle>()
        {
        }

        /*** Weight processing **/

        void resetWeightsUniformly()
        {
            for(auto it = this->begin(); it != this->end(); it++)
                it->weight_ = 1./this->size();
        }

        std::vector<float> getCumulatedWeights()
        {
            std::vector<float> cumulated_weights;
            cumulated_weights.push_back(this->begin()->weight_);
            unsigned int i = 0;
            for(auto it= this->begin()+1; it != this->end(); it++, ++i)
                cumulated_weights.push_back(cumulated_weights[i]  
                                            + it->weight_);
            return cumulated_weights;
        }

        void weigthsNormalization()
        {
            float sum_of_weights = sumOfWeights();
            for(auto it= this->begin(); it != this->end(); it++)
                it->weight_ /= sum_of_weights;
        }

        /*** Boxes processing **/

        void subdivise( unsigned int i = 0, 
                        SUBDIVISION_TYPE sub_type = SUBDIVISION_TYPE::RANDOM,
                        unsigned int N = 1, unsigned int dim = 0)
        {
            this->append(this->operator[](i).subdivise(sub_type, N, dim));
            this->erase(this->begin());
        }

        /*** Testing ***/

        bool wellPavedTest(IntervalVector initial_box)
        {
            if(this->size() > 0)
            {
                bool weak_disjoint = true;
                bool intersect;
                double intersect_volume;

                for(auto it= this->begin(); it != this->end()-1; it++)
                {
                    for(auto it1 = it+1; it1 != this->end(); it1++)
                    {
                        intersect = it->box_.intersects(it1->box_);
                        intersect_volume = (it->box_ & it1->box_).volume();
                        weak_disjoint = not intersect | intersect & intersect_volume == 0;
                        // We could use overlaps function that should give the right side 
                        // of the OR operator condition, but it seems that it doesn't work
                        if(!weak_disjoint) break;
                    }
                    if(!weak_disjoint) break;
                }
                EXPECT_TRUE(weak_disjoint)
                    << "All couple of boxes don't have zero volume intersection";

                IntervalVector reconstructed_initial_box(this->begin()->box_);
                double total_volume = this->begin()->box_.volume();
                for(auto it = this->begin()+1; it != this->end(); it++)
                {
                    reconstructed_initial_box = reconstructed_initial_box | it->box_;
                    total_volume += it->box_.volume();
                }
                bool no_outsiders = reconstructed_initial_box == initial_box;
                bool volume_equality = std::abs(total_volume - initial_box.volume()) < 1e-10;

                EXPECT_TRUE(no_outsiders)
                    << "There is no sub-box outside the initial one";
                EXPECT_TRUE(volume_equality) 
                    << "Total reconstructed volume should be equal to the initial one";

                return (no_outsiders & weak_disjoint & volume_equality);
            }
            else
            {
                return 0;
            }
        }

        #ifdef SUBDIVISE_OVER_ALL_DIMENSIONS
        bool subdiviseOverAllDimensionsTest()
        {
            bool equal_volume = true;
            bool hypercube = true;
            bool good_particles_number = true;
            for(unsigned int i = 0; i < this->size(); ++i)
            {
                if(this->operator[](0).box_.volume() != this->operator[](i).box_.volume()) 
                    equal_volume = false;

                for(unsigned int u = 0; u < this->operator[](i).box_.size(); ++u)
                    if(this->operator[](0).box_[0].diam() 
                        != this->operator[](i).box_[u].diam()) hypercube = false;
            }

            double i = std::log(this->size())
                        /std::log(pow(2,this->operator[](0).box_.size()));
            good_particles_number = std::fmod(i, double(1)) == 0;

            EXPECT_TRUE(good_particles_number)
                << "Wrong number of particles";
            EXPECT_TRUE(hypercube)    << "Each box should be an hypercube";
            EXPECT_TRUE(equal_volume) << "Volume of each box should be equal to the others";

            return good_particles_number & hypercube & equal_volume;
        }
        #endif

        #ifdef SUBDIVISE_OVER_GIVEN_DIRECTION
        bool subdiviseOverGivenDirectionTest(IntervalVector initial_box, unsigned int dir)
        {
            bool given_dir_well_subdivised, other_dirs_not_subdivised;
            bool test_succeed = true;
            unsigned int u = 0;
            for(auto it= this->begin(); it != this->end(); it++, ++u)
            {
                for(unsigned int dim = 0; dim < initial_box.size(); ++dim)
                {
                    if(dim != dir)
                    {
                        other_dirs_not_subdivised 
                            = std::abs(initial_box.diam()[dim] 
                                    - it->box_.diam()[dim]) < 1e-7;
                        if(!other_dirs_not_subdivised) test_succeed = false;
                        EXPECT_TRUE(other_dirs_not_subdivised) 
                            << "Another direction than " << dir 
                            << " subdivised in particle " << u << " : " << dim;
                    }
                    else
                    {
                        given_dir_well_subdivised 
                            = std::abs(it->box_.diam()[dim]
                                    -initial_box.diam()[dim]/this->size()) < 1e-7;
                        if(!given_dir_well_subdivised) test_succeed = false;
                        EXPECT_TRUE(given_dir_well_subdivised) 
                            << "subdivised direction " << dir 
                            << " as wrong diameter : expect " 
                            << initial_box.diam()[dim]/this->size() << " get " 
                            << it->box_.diam()[dim];
                    }
                }
            }

            return test_succeed;
        }
        #endif

        bool subdiviseOverRandomDimensionsTest
            (IntervalVector initial_box,
             std::map<int, std::pair<int, double>> geometrical_subdivision_map)
        {
            ROS_INFO_STREAM("Enter random dimension test");
            std::vector<std::vector<double>> normed_diameters;
            std::vector<double> vector_tmp;
            bool uniformly_subdivised = true;
            bool test_succeed = true;
            unsigned int u = 0;
            unsigned int o = 0;
            for(auto it = this->begin(); it != this->end(); it++, ++u)
            {
                vector_tmp.clear();
                o = 0;
                for(auto it1 = geometrical_subdivision_map.begin(); 
                         it1 != geometrical_subdivision_map.end();
                         ++it1)
                {
                    for(unsigned int i = 1; i < std::get<0>(it1->second); ++i)
                    {
                        uniformly_subdivised 
                            = std::abs((it->box_.diam()[it1->first+i] \
                                    - it->box_.diam()[(it1->first)]))\
                                    /std::get<1>(it1->second) < 1;
                        EXPECT_TRUE(uniformly_subdivised) 
                            << "Particle " << u << ", dimension " << o
                            << ": bad subdivision";
                        if(!uniformly_subdivised) test_succeed = false;
                        o++;
                    }
                }
                normed_diameters.push_back(vector_tmp);
            }
            ROS_INFO_STREAM("Finnish random dimension test");
            return test_succeed;
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
        unsigned int state_size_;                       // size of the boxes aka stat size
        float dt_;                                      // time step

        Particles particles_;

        //Random
        std::uniform_real_distribution<double> uniform_distribution_;

        // Dynamical model
        Variable state_variable_;
        IntervalVector* control_;
        Function* dynamics_model_;
        Function* measures_model_;

        // Prediction
        Particles predicted_particles_;
        Method integration_method_;
        double precision_;
        simulation* simu_;

        // Correction
        Particles corrected_particles_;
        std::vector<IntervalVector> innovation_;

        // Resampling
        Particles resampled_particles_;
        #if RESAMPLING_DIRECTION == 1
        std::map<int, std::pair<int, double>> geometrical_subdivision_map;
        #endif
        bool resampled_;

    protected:
        virtual void setDynamicalModel() = 0;
 
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

            while(current_particles_nb*pow(2,state_size_) <= N_)
            {
                for(unsigned int box_i = 0; box_i < current_particles_nb; ++box_i)
                    particles->subdivise(0, SUBDIVISION_TYPE::ALL_DIMESIONS, state_size_);
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
                Neff += 1./pow((*particles)[i].weight_, 2);
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
                    dir = getDirection(it->box_);
                    resampled_particles_
                        .append(it->subdivise(SUBDIVISION_TYPE::GIVEN, n[i], dir));
                }

                resampled_particles_.weigthsNormalization();
                ROS_DEBUG_STREAM("End resampling");
            }
            else{ ROS_DEBUG_STREAM("We don't resample"); }
        }

    public:
        /*** Box particle filter steps ***/

        BoxParticleFilter(  unsigned int N, unsigned int state_size, 
                            unsigned int control_size,
                            float dt, IntervalVector& initial_box): 
            state_variable_(state_size),
            control_(new IntervalVector(control_size)),
            uniform_distribution_(0.0,1.0)
        {
            ROS_ASSERT_MSG(state_size > 0, "State size has to be greater than 0");
            ROS_ASSERT_MSG(state_size == initial_box.size(), 
                    "State size and initial box size not consistent");

            N_          = N;
            state_size_ = state_size;

            dt_         = dt;
            integration_method_ = RK4;
            precision_ = 1e-6;

            initializeBoxes(initial_box);
        }

        void prediction(const IntervalVector control, 
                        BOXES_TYPE input_boxes_type = BOXES_TYPE::DEFAULT, 
                        bool ivp=false)
        {
            ROS_DEBUG_STREAM("prediction begin");
            Particles* particles = getParticlesPtr(input_boxes_type);
            predicted_particles_.clear();

            *control_ = control;

            ivp_ode problem = ivp_ode(*dynamics_model_, 0.0, IntervalVector(state_size_));

            IntervalVector predicted_box = IntervalVector(state_size_);

            for(auto it = particles->begin(); it == particles->end(); it++)
            {
                if(ivp)
                {
                    problem.yinit = &(it->box_);
                    delete simu_;
                    simu_ = new simulation(&problem, dt_, integration_method_, precision_);
                    simu_->run_simulation();
                    predicted_box = simu_->get_last();
                }
                else
                {
                    predicted_box = dynamics_model_->eval_vector(it->box_);
                }

                predicted_particles_.append(Particle(predicted_box, it->weight_));
            }

            if(ivp)
            {
                delete simu_;
                problem.yinit = NULL;
            }
            ROS_DEBUG_STREAM("prediction end");
        }

        void correction(const IntervalVector& measures,
                        BOXES_TYPE input_boxes_type = BOXES_TYPE::PREDICTION) 
        {   
            ROS_DEBUG_STREAM("Begin correction");
            innovation_.clear();
            Particles* particles = getParticlesPtr(BOXES_TYPE::RESAMPLING);
            corrected_particles_.clear();

            IntervalVector predicted_measures = IntervalVector(measures.size());
            IntervalVector innovation         = IntervalVector(measures.size());

            for(unsigned int i = 0; i < particles->size(); ++i)
            {
                predicted_measures  = measures_model_->eval_vector((*particles)[i].box_);
                innovation          = predicted_measures & measures;

                if(innovation.volume() > 0)
                {
                    innovation_.push_back(innovation);

                    corrected_particles_.append(Particle(contract(innovation_[i], 
                                                (*particles)[i].box_),
                                                (*particles)[i].weight_ 
                                                * (innovation.volume()
                                                /predicted_measures.volume()))); 
                }
            }

            // Resampling (if necessary)
            resampling(BOXES_TYPE::CORRECTION);

            ROS_DEBUG_STREAM("End correction");
        }

        /***  Testing ***/

        Particles getParticles(BOXES_TYPE boxes_type = BOXES_TYPE::DEFAULT)
        {
           return *getParticlesPtr(boxes_type); 
        }
};
