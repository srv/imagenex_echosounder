cmake_minimum_required(VERSION 2.8.3)
project(ecosonda)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cmake_modules
)


#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   #INCLUDE_DIRS include
#  LIBRARIES ecosonda
  #CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#add_library(ecosonda src/TimeoutSerial.cpp)



add_executable(ecosonda src/ecosonda_node.cpp
			src/TimeoutSerial.cpp)

target_link_libraries(ecosonda ${catkin_LIBRARIES}
			       ${Boost_LIBRARIES})


