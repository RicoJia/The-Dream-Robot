cmake_minimum_required(VERSION 3.0.2)
project(dream_base_description)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
        # nuturtlebot
)

find_package(
        gazebo REQUIRED
        )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)


# TODO: gazebo plugins
# add_library(${PROJECT_NAME}_plugin SHARED src/turtle_drive_plugin.cpp)
# target_link_libraries(${PROJECT_NAME}_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# target_include_directories(${PROJECT_NAME}_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
# add_dependencies(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


