cmake_minimum_required(VERSION 2.8.3)
project(psywerx)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs pcl pcl_ros sensor_msgs)

catkin_package(
#   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster src/cluster.cpp)
target_link_libraries(cluster ${catkin_LIBRARIES})

add_executable(floor src/floor.cpp)
target_link_libraries(floor ${catkin_LIBRARIES})

add_executable(hyper src/hyper.cpp)
target_link_libraries(hyper ${catkin_LIBRARIES})

add_executable(cylinder src/cylinder_segmentation.cpp)
target_link_libraries(cylinder ${catkin_LIBRARIES})

add_executable(marko src/marko.cpp)
#set_target_properties(marko PROPERTIES COMPILE_FLAGS -DDEBUG)
target_link_libraries(marko ${catkin_LIBRARIES})
