cmake_minimum_required(VERSION 3.9)
project(bpf_localization)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-fopenmp)
add_compile_options(-std=c++11)
add_compile_options(-frounding-math -O1)
#add_compile_options(-Wall)

SET(IBEX_DEPENDENCIES  "/usr/local/lib/libibex.a" "/usr/local/lib/libprim.a")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  rostest
  interval_msgs
  std_msgs
  nav_msgs
  message_generation
)

find_package(OpenMP REQUIRED)
#find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
   FILES
   IntervalIMU.msg
 )

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
   FILES
   StartCalibration.srv
   GetDiameters.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   interval_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  std_msgs
  interval_msgs
#  LIBRARIES bpf_localization
#  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/bpf_localization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(boat_bpf_localization src/boat_bpf_localization.cpp)
 add_executable(imu_standalone src/imu_standalone.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(boat_bpf_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(imu_standalone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(boat_bpf_localization
    ${catkin_LIBRARIES}
    ${IBEX_DEPENDENCIES}
)

target_link_libraries(imu_standalone 
    ${catkin_LIBRARIES}
    ${IBEX_DEPENDENCIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Gtests unit tests

catkin_add_gtest(bpf_particle_given_dir_subdivision tests/bpf_particle_given_dir_subdivision.cpp)
if(TARGET bpf_particle_given_dir_subdivision)
    target_link_libraries(bpf_particle_given_dir_subdivision
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(bpf_particle_uniform_subdivision tests/bpf_particle_uniform_subdivision.cpp)
if(TARGET bpf_particle_uniform_subdivision)
    target_link_libraries(bpf_particle_uniform_subdivision
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(bpf_particle_random_subdivision tests/bpf_particle_random_subdivision.cpp)
if(TARGET bpf_particle_random_subdivision)
    target_link_libraries(bpf_particle_random_subdivision
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(bpf_resampling_multinomial_tests tests/bpf_resampling_multinomial_tests.cpp)
if(TARGET bpf_resampling_multinomial_tests)
    target_link_libraries(bpf_resampling_multinomial_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(   bpf_resampling_guaranted_geometrical_tests 
                    tests/bpf_resampling_guaranted_geometrical_tests.cpp)
if(TARGET bpf_resampling_guaranted_geometrical_tests)
    target_link_libraries(bpf_resampling_guaranted_geometrical_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(bpf_generic_tests tests/bpf_generic_tests.cpp)
if(TARGET bpf_generic_tests)
    target_link_libraries(bpf_generic_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(bpf_init_tests_uniform_paving tests/bpf_init_tests_uniform_paving.cpp)
if(TARGET bpf_init_tests_uniform_paving)
    target_link_libraries(bpf_init_tests_uniform_paving
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(   bpf_init_tests_uniformly_choosen_paving 
                    tests/bpf_init_tests_uniformly_choosen_paving.cpp)
if(TARGET bpf_init_tests_uniformly_choosen_paving)
    target_link_libraries(bpf_init_tests_uniformly_choosen_paving
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(turtlebot_tests tests/dynamical_systems/turtlebot_tests.cpp)
if(TARGET turtlebot_tests)
    target_link_libraries(turtlebot_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(   guaranted_integration_tests
                    tests/dynamical_systems/guaranted_integration_tests.cpp)
if(TARGET guaranted_integration_tests)
    target_link_libraries(guaranted_integration_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(   non_guaranted_interval_HEUN_integration_tests
                    tests/dynamical_systems/non_guaranted_interval_HEUN_integration_tests.cpp)
if(TARGET non_guaranted_interval_HEUN_integration_tests)
    target_link_libraries(non_guaranted_interval_HEUN_integration_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(   non_guaranted_affine_HEUN_integration_tests
                    tests/dynamical_systems/non_guaranted_affine_HEUN_integration_tests.cpp)
if(TARGET non_guaranted_affine_HEUN_integration_tests)
    target_link_libraries(non_guaranted_affine_HEUN_integration_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(   imu_dynamical_model_tests 
                    tests/dynamical_systems/imu_dynamical_model_tests.cpp)
if(TARGET imu_dynamical_model_tests)
    target_link_libraries(imu_dynamical_model_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(   boat_bpf_localization_tests tests/boat_bpf_localization_tests.cpp)
if(TARGET boat_bpf_localization_tests)
    target_link_libraries(boat_bpf_localization_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(functions_tests tests/dynamical_systems/functions_tests.cpp)
if(TARGET functions_tests)
    target_link_libraries(functions_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

catkin_add_gtest(bpf_particle_weight_tests tests/bpf_particle_weight_tests.cpp)
if(TARGET bpf_particle_weight_tests)
    target_link_libraries(bpf_particle_weight_tests
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

## ROSTESTS

add_rostest_gtest(tests_imu_standalone rostests/imu_standalone.test tests/rostest_imu_standalone.cpp)
if(TARGET tests_imu_standalone)
    target_link_libraries(tests_imu_standalone 
        ${catkin_LIBRARIES}
        ${IBEX_DEPENDENCIES}
    )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
