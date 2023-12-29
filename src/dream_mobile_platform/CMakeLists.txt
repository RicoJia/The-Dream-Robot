cmake_minimum_required(VERSION 3.0.2)
project(dream_mobile_platform)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ToggleLED.srv
)


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dream_mobile_platform
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    file(GLOB ${PROJECT_NAME}_UNIT_TEST_FILES tests/unit/*.py)
    foreach(_test_file ${${PROJECT_NAME}_UNIT_TEST_FILES})
        catkin_add_nosetests(${_test_file})
    endforeach()
endif()