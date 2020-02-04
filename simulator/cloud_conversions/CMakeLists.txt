cmake_minimum_required(VERSION 2.8.3)
project(cloud_conversions)

find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	roscpp
	sensor_msgs
)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

