cmake_minimum_required(VERSION 2.8.3)
project(psywerx)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
