cmake_minimum_required(VERSION 3.0.2)
project(dream_gmapping)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)
# find_package(simple_robotics_cpp_utils REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dream_gmapping
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(${PROJECT_NAME} src/dream_gmapping.cpp)
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
    /usr/local/
)
# target_link_libraries(${PROJECT_NAME} PRIVATE simple_robotics_cpp_utils)



