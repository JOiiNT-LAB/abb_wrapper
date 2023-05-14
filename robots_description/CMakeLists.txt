cmake_minimum_required(VERSION 3.0.2)
project(robots_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime roslib
)

###########
## Build ##
###########

# install(DIRECTORY config launch meshes xacro
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})