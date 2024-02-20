# run cmake-format -i CMakeLists.txt to reformat
cmake_minimum_required(VERSION 3.0.2)
project(dream_gmapping)

add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS # Needed for ros nodehandle, etc. Note: roscpp is the COMPONENT,
             # and catkin is required.
             roscpp tf2_ros geometry_msgs)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
catkin_package()

# ##############################################################################
# Build ##
# ##############################################################################

file(GLOB src_files "src/*.cpp")
add_executable(${PROJECT_NAME} ${src_files})
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  # Technically don't need this, but want to make it clear that we include
  # simple_robotics_cpp_utils in /usr/local/include
  ${CMAKE_SYSTEM_PREFIX_PATH}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen
                      ${PCL_LIBRARIES})

# Unit Tests Then one can run these tests using catkin_make run_tests or
# catkin_make run_tests --pkg <PKG_NAME> file(GLOB test_src_files
# "tests/unit/*.cpp")
catkin_add_gtest(test_particle_filter tests/unit/test_particle_filter.cpp)
catkin_add_gtest(test_dream_gmapper tests/unit/test_dream_gmapper.cpp
                 src/dream_gmapper.cpp)
target_link_libraries(test_particle_filter ${catkin_LIBRARIES} Eigen3::Eigen)
target_link_libraries(test_dream_gmapper ${catkin_LIBRARIES} Eigen3::Eigen
                      ${PCL_LIBRARIES})
