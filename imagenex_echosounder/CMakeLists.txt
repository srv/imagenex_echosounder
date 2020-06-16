cmake_minimum_required(VERSION 2.8.3)
project(imagenex_echosounder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cmake_modules
)


#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   #INCLUDE_DIRS include
#  LIBRARIES imagenex_echosounder
  #CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#add_library(imagenex_echosounder src/TimeoutSerial.cpp)



add_executable(imagenex_echosounder src/imagenex_echosounder_node.cpp
			src/TimeoutSerial.cpp)

target_link_libraries(imagenex_echosounder ${catkin_LIBRARIES}
			       ${Boost_LIBRARIES})


