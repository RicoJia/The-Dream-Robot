cmake_minimum_required(VERSION 3.0.2)
project(dream_gmapping)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    # Needed for ros nodehandle, etc.
    # Note: roscpp is the COMPONENT, and catkin is required.
    roscpp
    tf2_ros
    geometry_msgs
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
catkin_package()

###########
## Build ##
###########

add_executable(${PROJECT_NAME} src/dream_gmapping.cpp)
include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    # Technically don't need this, but want to make it clear that 
    # we include simple_robotics_cpp_utils in /usr/local/include
    ${CMAKE_SYSTEM_PREFIX_PATH}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


# Unit Tests
# Then one can run these tests using catkin_make run_tests or catkin_make run_tests --pkg <PKG_NAME>
file(GLOB test_src_files "tests/unit/*.cpp")
catkin_add_gtest(${PROJECT_NAME}_test ${test_src_files})