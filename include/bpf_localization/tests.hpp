#ifndef TESTS
#define TESTS

#include <bpf_localization/box_particle_filter.hpp>

using namespace bpf;

bool compareParticles(const Particles* particles1, const Particles* particles2)
{
    bool found_particle, already_found;
    std::vector<unsigned int> found_particles;
    unsigned int i;
    for(auto part1 = particles1->begin(); part1 != particles1->end(); ++part1)
    {
        i = 0;
        found_particle = false;
        for(auto part2 = particles2->begin(); part2 != particles2->end(); ++part2)
        {
            if(eps_equals(*part1,*part2, 1e-10))
            {
                already_found = false;
                for(unsigned int u = 0; u < found_particles.size(); u++)
                {
                    if(found_particles[u] == i)
                    {
                        already_found = true;
                        break;
                    }
                }
                
                if(!already_found)
                {
                    found_particle = true;
                    found_particles.push_back(i);
                    break;
                }
            }

            i++;
        }

        if(!found_particle) return false;    
    }

    //for(unsigned int u = 0; u < found_particles.size(); u++)
    //    ROS_INFO_STREAM(found_particles[u]);

    return true;
}

bool wellPavedTest(Particles* particles, IntervalVector initial_box)
{
    if(particles->size() > 0)
    {
        bool weak_disjoint = true;
        bool intersect;
        double intersect_volume;

        for(auto it= particles->begin(); it != particles->end()-1; it++)
        {
            for(auto it1 = it+1; it1 != particles->end(); it1++)
            {
                intersect = it->intersects(*it1);
                intersect_volume = (*it & *it1).volume();
                weak_disjoint = not intersect | intersect & intersect_volume == 0;
                // We could use overlaps function that should give the right side 
                // of the OR operator condition, but it seems that it doesn't work
                if(!weak_disjoint) break;
            }
            if(!weak_disjoint) break;
        }
        EXPECT_TRUE(weak_disjoint)
            << "All couple of boxes don't have zero volume intersection";

        IntervalVector reconstructed_initial_box(*(particles->begin()));
        double total_volume = particles->begin()->volume();
        for(auto it = particles->begin()+1; it != particles->end(); it++)
        {
            reconstructed_initial_box = reconstructed_initial_box | *it;
            total_volume += it->volume();
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
bool subdiviseOverAllDimensionsTest(Particles* particles)
{
    bool equal_volume = true;
    bool hypercube = true;
    bool good_particles_number = true;
    for(unsigned int i = 0; i < particles->size(); ++i)
    {
        if(particles->operator[](0).volume() != particles->operator[](i).volume()) 
            equal_volume = false;

        for(unsigned int u = 0; u < particles->operator[](i).size(); ++u)
            if(particles->operator[](0)[0].diam() 
                != particles->operator[](i)[u].diam()) hypercube = false;
    }

    double i = std::log(particles->size())
                /std::log(pow(2,particles->operator[](0).size()));
    good_particles_number = std::fmod(i, double(1)) == 0;

    EXPECT_TRUE(good_particles_number)
        << "Wrong number of particles";
    EXPECT_TRUE(hypercube)    << "Each box should be an hypercube";
    EXPECT_TRUE(equal_volume) << "Volume of each box should be equal to the others";

    return good_particles_number & hypercube & equal_volume;
}
#endif

#ifdef SUBDIVISE_OVER_GIVEN_DIRECTION
bool subdiviseOverGivenDirectionTest(Particles* particles, 
                                     IntervalVector initial_box, 
                                     unsigned int dir)
{
    bool given_dir_well_subdivised, other_dirs_not_subdivised;
    bool test_succeed = true;
    unsigned int u = 0;
    for(auto it= particles->begin(); it != particles->end(); it++, ++u)
    {
        for(unsigned int dim = 0; dim < initial_box.size(); ++dim)
        {
            if(dim != dir)
            {
                other_dirs_not_subdivised 
                    = std::abs(initial_box.diam()[dim] 
                            - it->diam()[dim]) < 1e-7;
                if(!other_dirs_not_subdivised) test_succeed = false;
                EXPECT_TRUE(other_dirs_not_subdivised) 
                    << "Another direction than " << dir 
                    << " subdivised in particle " << u << " : " << dim;
            }
            else
            {
                given_dir_well_subdivised 
                    = std::abs(it->diam()[dim]
                            -initial_box.diam()[dim]/particles->size()) < 1e-7;
                if(!given_dir_well_subdivised) test_succeed = false;
                EXPECT_TRUE(given_dir_well_subdivised) 
                    << "subdivised direction " << dir 
                    << " as wrong diameter : expect " 
                    << initial_box.diam()[dim]/particles->size() << " get " 
                    << it->diam()[dim];
            }
        }
    }

    return test_succeed;
}
#endif

bool subdiviseOverRandomDimensionsTest
    (Particles* particles, IntervalVector initial_box,
     std::map<int, std::pair<int, double>> geometrical_subdivision_map)
{
    ROS_INFO_STREAM("Enter random dimension test");
    std::vector<std::vector<double>> normed_diameters;
    std::vector<double> vector_tmp;
    bool uniformly_subdivised = true;
    bool test_succeed = true;
    unsigned int u = 0;
    unsigned int o = 0;
    for(auto it = particles->begin(); it != particles->end(); it++, ++u)
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
                    = std::abs((it->diam()[it1->first+i] \
                            - it->diam()[(it1->first)]))\
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

#endif
