cmake_minimum_required(VERSION 2.8.3)
project(imagenex_echosounder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cmake_modules
)


#find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(HEADER_FILES
    "${CMAKE_CURRENT_LIST_DIR}/src/include/TimeoutSerial.h")

add_executable(imagenex_echosounder src/imagenex_echosounder_node.cpp src/TimeoutSerial.cpp)
target_link_libraries(imagenex_echosounder ${catkin_LIBRARIES})
add_dependencies(imagenex_echosounder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})