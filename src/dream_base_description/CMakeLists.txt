cmake_minimum_required(VERSION 3.0.2)
project(dream_base_description)

# By default, ROS noetic has C++14. But on a package level, you can do C++17
# Compile as C++17, supported in ROS noetic and newer
add_compile_options(-std=c++17)

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


# gazebo plugins
add_library(dream_gazebo_diff_drive_plugin SHARED src/dream_gazebo_diff_drive_plugin.cpp)
target_link_libraries(dream_gazebo_diff_drive_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

target_include_directories(dream_gazebo_diff_drive_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
add_dependencies(dream_gazebo_diff_drive_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


