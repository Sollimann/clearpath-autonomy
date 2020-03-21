cmake_minimum_required(VERSION 2.8.3)
project(jackal_description)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
    cv_bridge
    rostest
)

catkin_package(
    DEPENDS
        gazebo_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(WIN32)
install(PROGRAMS scripts/env_run.bat
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else()
install(PROGRAMS scripts/env_run
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()
