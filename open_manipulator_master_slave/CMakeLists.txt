################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_master_slave)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    robotis_manipulator
    open_manipulator_libs
    open_manipulator_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp robotis_manipulator open_manipulator_libs open_manipulator_msgs

)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_manipulator_master src/open_manipulator_master.cpp)
add_dependencies(open_manipulator_master ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_master ${catkin_LIBRARIES} )

# custum by suho, for recording trajectory and making these data to txt

add_executable(open_manipulator_recording_trajectory src/open_manipulator_recording_trajectory.cpp)

add_dependencies(open_manipulator_recording_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(open_manipulator_recording_trajectory ${catkin_LIBRARIES} )

# custum by suho, for play recorded trajectory

add_executable(open_manipulator_play_recorded_trajectory src/open_manipulator_play_recorded_trajectory.cpp)

add_dependencies(open_manipulator_play_recorded_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(open_manipulator_play_recorded_trajectory ${catkin_LIBRARIES} )

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_master
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# custum by suho, for recording trajectory and making these data to txt

install(TARGETS open_manipulator_recording_trajectory

  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)

# custum by suho, for play recorded trajectory

install(TARGETS open_manipulator_play_recorded_trajectory

  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



################################################################################
# Test
################################################################################
