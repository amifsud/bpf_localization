#include "ibex/ibex.h"
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <numeric>
#include <random>

using namespace ibex;

class BoxParticleFilter
{
    protected:
        unsigned int N_;                                // Number of particles
        unsigned int state_size_;                       // size of the boxes aka stat size
        float dt_;                                      // time step

        std::vector<IntervalVector> boxes_;             // boxes
        std::vector<float> weights_;                    // weightis

        //Random
        std::uniform_real_distribution<double> uniform_distribution_;

        // Dynamical model
        Variable state_variable_;
        IntervalVector* control_;
        Function* dynamics_model_;
        Function* measures_model_;

        // Prediction
        std::vector<IntervalVector> predicted_boxes_;
        std::vector<float> predicted_weights_;       
        Method integration_method_;
        double precision_;
        simulation* simu_;

        // Correction
        std::vector<IntervalVector> innovation_;
        std::vector<IntervalVector> corrected_boxes_;
        std::vector<float> corrected_weights_;

        // Resampling
        bool resampled_;
        std::vector<IntervalVector> resampled_boxes_;
        std::vector<float> resampled_weights_;

    protected:
        virtual void setDynamicalModel() = 0;

        #ifdef UNIFORM_PAVING_INIT
        std::vector<IntervalVector> subdiviseOverAllDimensions(IntervalVector box, 
                                                               unsigned int dim = 0)
        {
            std::vector<IntervalVector> vector, vector_tmp;
            std::pair<IntervalVector, IntervalVector> pair = box.bisect(dim); 

            if(dim < state_size_ - 1)
            {
                vector     = subdiviseOverAllDimensions(std::get<0>(pair), dim+1);
                vector_tmp = subdiviseOverAllDimensions(std::get<1>(pair), dim+1);
                vector.insert(vector.end(), vector_tmp.begin(), vector_tmp.end()); 
            }
            else
            {
                vector.push_back(std::get<0>(pair));
                vector.push_back(std::get<1>(pair));
            }
            return vector;
        }
        #endif

        std::vector<IntervalVector> subdiviseOverRandomDimensions(  IntervalVector box, 
                                                                    unsigned int N)
        {
            std::vector<IntervalVector> boxes;
            std::pair<IntervalVector, IntervalVector> pair = box.bisect(0);
            if(N > 1) boxes.push_back(box);
            unsigned int boxes_nb = 1;
            unsigned int direction;

            while (boxes_nb < N)
            {
                std::default_random_engine generator;
                direction = int(uniform_distribution_(generator) * state_size_);
                pair = boxes[0].bisect(direction); 
                boxes.push_back(std::get<0>(pair));
                boxes.push_back(std::get<1>(pair));
                boxes.erase(boxes.begin());
			    boxes_nb += 1;
            }

            return boxes;
        }

        #ifdef UNIFORM_PAVING_INIT
        void initializeBoxes(IntervalVector initial_box)
        {
            ROS_DEBUG_STREAM("Uniform paving initialization begin");

            boxes_.clear();
            boxes_.push_back(initial_box);
            unsigned int current_boxes_nb = 1;
            std::vector<IntervalVector> vector_tmp;

            while(current_boxes_nb*pow(2,state_size_) <= N_)
            {
                for(unsigned int box_i = 0; box_i < current_boxes_nb; ++box_i)
                {
                    vector_tmp = subdiviseOverAllDimensions(boxes_[0]);
                    boxes_.insert(boxes_.end(), vector_tmp.begin(), vector_tmp.end());
                    boxes_.erase(boxes_.begin());
                }
                current_boxes_nb = boxes_.size();
            }
            ROS_DEBUG_STREAM("Uniform paving initialization end");
        }
        #endif

        #ifdef UNIFORMLY_CHOOSEN_PAVING_INIT
        void initializeBoxes(IntervalVector initial_box)
        {
            ROS_DEBUG_STREAM("Uniformly choosen paving initialization begin");
            // We choose to subdvise the initial box with equal size boxes (1)

            boxes_.clear();
            boxes_ = subdiviseOverRandomDimensions(initial_box, N_);

            ROS_DEBUG_STREAM("Uniformly choosen paving initialization end");
        }
        #endif

    public:
        BoxParticleFilter(  unsigned int N, unsigned int state_size, 
                            unsigned int control_size,
                            float dt, IntervalVector& initial_box): 
            weights_(N, 1.0/N), 
            state_variable_(state_size),
            control_(new IntervalVector(control_size)),
            uniform_distribution_(0.0,1.0)
        {
            dt_         = dt;
            N_          = N;
            state_size_ = state_size;

            integration_method_ = RK4;
            precision_ = 1e-6;

            if(state_size != initial_box.size()) 
                ROS_ERROR("wrong intial box size, different than state size");

            initializeBoxes(initial_box);
        }

        std::vector<IntervalVector> getBoxes()
        {
            return boxes_;
        }
 
        void prediction(const IntervalVector control, bool ivp=false)
        {
            ROS_DEBUG_STREAM("prediction begin");
            predicted_boxes_.clear();
            predicted_weights_.clear();

            *control_ = control;

            ivp_ode problem = ivp_ode(*dynamics_model_, 0.0, IntervalVector(state_size_));

            IntervalVector predicted_box = IntervalVector(state_size_);

            for(unsigned int i = 0; i < boxes_.size(); ++i)
            {
                if(ivp)
                {
                    problem.yinit = &boxes_[i];
                    delete simu_;
                    simu_ = new simulation(&problem, dt_, integration_method_, precision_);
                    simu_->run_simulation();
                    predicted_box = simu_->get_last();
                }
                else
                {
                    predicted_box = dynamics_model_->eval_vector(boxes_[i]);
                }

                predicted_boxes_.push_back(predicted_box);
                predicted_weights_.push_back(weights_[i]);
            }

            if(ivp)
            {
                delete simu_;
                problem.yinit = NULL;
            }
            ROS_DEBUG_STREAM("prediction end");
        }

        IntervalVector contract(IntervalVector innovation, IntervalVector& box)
        {
            // Use a contractor to found the box subset that give the innovation 
            // (i.e. (predicted measure)s & (measures)) by the measures dynamics
            return box;
        }


        #ifdef MULTINOMIAL_RESAMPLING
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

        #ifdef GUARANTED_RESAMPLING
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

        void resampling()
        {
            ROS_DEBUG_STREAM("Will we resample");
            resampled_boxes_.clear();
            resampled_weights_.clear();

            // Check that we need to resample
            double Neff = 0;
            for(unsigned int i = 0; i < boxes_.size(); ++i)
                Neff += 1./pow(weights_[i], 2);
            if(true)//Neff <= 10 * boxes_.size())
            {
                ROS_DEBUG_STREAM("We will resample");

                // Compute cumulated weights
                std::vector<float> cumulated_weights;
                cumulated_weights.push_back(weights_[0]);
                for(unsigned int i = 1; i < boxes_.size(); ++i)
                    cumulated_weights.push_back(cumulated_weights[i-1]  
                                                + weights_[i]);

                // Compute number of subdivisions per boxes
                std::vector<unsigned int> n = chooseSubdivisions(cumulated_weights);

                // Subdivise boxes with ni boxes (delete box if ni=0) 
                std::vector<IntervalVector> subdivided_box; 
                for(unsigned int i = 0; i < boxes_.size(); i++)
                {
                    subdivided_box = subdiviseOverRandomDimensions( boxes_[i], 
                                                                    n[i]); 
                    resampled_boxes_.insert(resampled_boxes_.end(), subdivided_box.begin(), 
                                            subdivided_box.end());
                }

                // Reset weights
                resampled_weights_ = std::vector<float>(N_, 1.0/N_); 
 
                ROS_DEBUG_STREAM("End resampling");
            }
            else{ ROS_DEBUG_STREAM("We don't resample"); }
        }

        void correction(const IntervalVector& measures)
        {   
            ROS_DEBUG_STREAM("Begin correction");
            innovation_.clear();
            corrected_boxes_.clear();
            corrected_weights_.clear();

            IntervalVector predicted_measures = IntervalVector(measures.size());
            IntervalVector innovation         = IntervalVector(measures.size());

            for(unsigned int i = 0; i < boxes_.size(); ++i)
            {
                predicted_measures  = measures_model_->eval_vector(predicted_boxes_[i]);
                innovation          = predicted_measures & measures;

                if(innovation.volume() > 0)
                {
                    innovation_.push_back(innovation);

                    corrected_boxes_
                        .push_back(contract(innovation_[i], predicted_boxes_[i]));

                    corrected_weights_.push_back(predicted_weights_[i] 
                            * (innovation.volume()/predicted_measures.volume())); 
                                // Likelihood
                }
            }

            // Weights normalization
            double sum_of_weights = std::accumulate(corrected_weights_.begin(), 
                                                    corrected_weights_.end(),
                                                    decltype(corrected_weights_)
                                                        ::value_type(0));
            for(unsigned int i = 0; i < corrected_boxes_.size(); i++)
                corrected_weights_[i] /= sum_of_weights;
            ROS_DEBUG_STREAM("End correction");
        }
};
