cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generators)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    rtt_ros_tools
    roscpp
    rtt_ros
    rtt_roscomm
    rtt_rosclock
    rospy
    trac_ik_lib
    tf
    kdl_conversions
    rtt_geometry_msgs
)

catkin_python_setup()

find_package(OROCOS-RTT REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
      include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

# Orocos typekits
ros_generate_rtt_typekit(trajectory_generators)

set(COMPONENT_SOURCES
	src/random_waypoint_generator.cpp
	src/random_waypoint_array_generator.cpp
        src/cartesian_trajectory_generator.cpp
        src/joint_space_velocity_profiles.cpp
)
set(PYTHON_SCRIPTS
    src/Pose_publisher1.py
    src/Pose_publisher2.py
    src/PoseArray_publisher1.py
    src/PoseArray_publisher2.py
)
## Orocos component
orocos_component(${PROJECT_NAME} ${COMPONENT_SOURCES})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
   ${trac_ik_lib_LIBRARIES}
)

## Generate install targets for header files
orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)

catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
