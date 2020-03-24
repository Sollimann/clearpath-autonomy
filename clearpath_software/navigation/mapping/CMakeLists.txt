cmake_minimum_required(VERSION 2.8.3)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
 map_server
)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/publish_map.launch)
endif()

install(DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
