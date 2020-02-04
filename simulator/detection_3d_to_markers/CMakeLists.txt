cmake_minimum_required(VERSION 2.8.3)
project(detection_3d_to_markers)
set (CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	visualization_msgs
	vision_msgs
)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/detection_3d_to_markers.cpp
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/detection_3d_to_markers_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

