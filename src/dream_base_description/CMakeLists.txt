# CMAKE_EXPORT_COMPILE_COMMANDS requires 3.5+
cmake_minimum_required(VERSION 3.5)
project(dream_base_description)

# Convert the environment variable to upper case, and skip if 
# This environment variable is set
string(TOUPPER $ENV{SKIP_OFFLINE_PACKAGES} SKIP_OFFLINE_PACKAGES_UPPER)
message(STATUS "SKIP_OFFLINE_PACKAGES is ${SKIP_OFFLINE_PACKAGES_UPPER}")
if(SKIP_OFFLINE_PACKAGES_UPPER STREQUAL "TRUE")
  message(STATUS "Skipping ${PROJECT_NAME} due to SKIP_OFFLINE_PACKAGES variable.")
  return()
endif()

# By default, ROS noetic has C++14. But on a package level, you can do C++17
# Compile as C++17, supported in ROS noetic and newer
add_compile_options(-std=c++17)

# equivalent to setting -DCMAKE_EXPORT_COMPILE_COMMANDS=1 once built, in /build,
# will see compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(
  catkin REQUIRED COMPONENTS roscpp gazebo_ros tf2_ros
                             # nuturtlebot
)

find_package(gazebo REQUIRED)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
  # we include simple_robotics_cpp_utils in /usr/local/include
  ${CMAKE_SYSTEM_PREFIX_PATH})

# gazebo plugins
add_library(dream_gazebo_diff_drive_plugin SHARED
            src/dream_gazebo_diff_drive_plugin.cpp)
target_link_libraries(dream_gazebo_diff_drive_plugin ${GAZEBO_LIBRARIES}
                      ${catkin_LIBRARIES})

target_include_directories(dream_gazebo_diff_drive_plugin
                           PUBLIC ${GAZEBO_INCLUDE_DIRS})
add_dependencies(dream_gazebo_diff_drive_plugin
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
