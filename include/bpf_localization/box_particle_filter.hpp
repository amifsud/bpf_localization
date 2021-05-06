#include "ibex/ibex.h"
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <numeric>
#include <random>

/*** Available algos ***/
// Init
//  * uniformly choosen : 0
//  * uniform           : 1
// Resampling
//  * multinomial       : 0
//  * guaranted         : 1

/*** Defaults algos versions ***/
#ifndef RESAMPLING_METHOD
    #define RESAMPLING_METHOD   0
#endif

#ifndef INIT_METHOD
    #define INIT_METHOD         0
#endif

/*** Algos dependencies ***/
#if INIT_METHOD == 1
    #define SUBDIVISE_OVER_ALL_DIMENSIONS
#endif

using namespace ibex;

enum BOXES_TYPE{PREDICTION, CORRECTION, RESAMPLING, DEFAULT};

class Particle
{
    protected:
        // Random
        std::uniform_real_distribution<double> uniform_distribution_;

        // For subdivisions
        std::vector<Particle> boxes, boxes_tmp;
        std::pair<IntervalVector, IntervalVector> pair;
        unsigned int direction;

    public:
        IntervalVector box_;
        float weight_;
 
    protected:
        /*** Boxes processing ***/

        #ifdef SUBDIVISE_OVER_ALL_DIMENSIONS
        std::vector<Particle> subdiviseOverAllDimensions(unsigned int dim = 0)
        {
            boxes.clear();
            boxes_tmp.clear();

            //pair = this->box_.bisect(dim); 

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
            return boxes;
        }
        #endif

        std::vector<Particle> subdiviseOverRandomDimensions(unsigned int N)
        {
            boxes.clear();
            boxes.push_back(*this);

            while (boxes.size() < N)
            {
                std::default_random_engine generator;
                direction = int(uniform_distribution_(generator) * this->box_.size());
                pair = boxes[0].box_.bisect(direction, 0.5); 
                boxes.push_back(Particle(std::get<0>(pair), boxes[0].weight_/2.));
                boxes.push_back(Particle(std::get<1>(pair), boxes[0].weight_/2.));
                boxes.erase(boxes.begin());
            }

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

        std::vector<Particle> subdivise(unsigned int N = 0, bool force_random = false)
        {
            std::vector<Particle> particles;

            if(!force_random)
            {
                #ifdef SUBDIVISE_OVER_ALL_DIMENSIONS
                particles = subdiviseOverAllDimensions();
                #else
                particles = subdiviseOverRandomDimensions(N);
                #endif
            }
            else
            {
                particles = subdiviseOverRandomDimensions(N);
            }

            return particles;
        }
};

class Particles: public std::vector<Particle>
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

        Particles(std::vector<Particle> particles): std::vector<Particle>(particles)
        {
        }

        Particles(): std::vector<Particle>()
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

        void subdivise(unsigned int i, unsigned int N = 0, bool force_random = false)
        {
            this->append(this->operator[](i).subdivise(N, force_random));
            this->erase(this->begin());
        }

        /*** Testing ***/

        bool wellPavedTest(IntervalVector initial_box)
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

        /*** Appending ***/

        void append(std::vector<Particle> particles)
        {
            this->insert(this->end(), particles.begin(), particles.end());
        }

        void append(std::vector<IntervalVector> boxes, double weight)
        {
            append(boxes, std::vector<double>(boxes.size(), weight));
        }

        void append(std::vector<IntervalVector> boxes, std::vector<double> weights)
        {
            for(unsigned int i = 0; i < boxes.size(); ++i)
                this->append(boxes[i], weights[i]);
        }

        void append(const IntervalVector box_in, const double weight_in)
        {
            this->insert(this->end(), Particle(box_in, weight_in));
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
            particles->append(initial_box, 1.);
            particles->subdivise(0, N_);
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
            particles->append(initial_box, 1.);

            unsigned int current_particles_nb = 1;

            while(current_particles_nb*pow(2,state_size_) <= N_)
            {
                for(unsigned int box_i = 0; box_i < current_particles_nb; ++box_i)
                    particles->subdivise(0, state_size_);
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
            chooseSubdivisions(const std::vector<float>& cumulated_weights)
        {
            ROS_DEBUG_STREAM("Begin multinomial resampling");
            // Multinomial resampling
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
            chooseSubdivisions(const std::vector<float>& cumulated_weights)
        {
            ROS_DEBUG_STREAM("Begin guaranted resampling");
            // Guaranted resampling
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

            ROS_DEBUG_STREAM("End guaranted resampling");
            return n;
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
            if(Neff <= 10 * particles->size())
            {
                ROS_DEBUG_STREAM("We will resample");

                // Compute number of subdivisions per boxes
                std::vector<unsigned int> n 
                    = chooseSubdivisions(particles->getCumulatedWeights());

                // Subdivise boxes with ni boxes (delete box if ni=0) 
                unsigned int i = 0;
                for(auto it = particles->begin(); it == particles->end(); it++, i++)
                    resampled_particles_.append(it->subdivise(n[i], true));

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

                predicted_particles_.append(predicted_box, it->weight_);
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

                    corrected_particles_.append(contract(innovation_[i], 
                                                (*particles)[i].box_),
                                                (*particles)[i].weight_ 
                                                * (innovation.volume()/predicted_measures.volume())); 
                                                // Likelihood
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
