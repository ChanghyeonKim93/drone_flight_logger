cmake_minimum_required(VERSION 2.8.3)
project(drone_flight_logger)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  message_filters

  sensor_msgs
  std_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
		DEPENDS Eigen
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/DroneFlightLogger.cpp)
set(SRC ${SRC} include/DroneFlightLogger.h)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

add_executable (drone_flight_logger_node ${SRC} )
target_link_libraries (drone_flight_logger_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries (drone_flight_logger_node ${Eigen_LIBRARIES} )
