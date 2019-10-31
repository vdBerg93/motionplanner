cmake_minimum_required(VERSION 2.8.3)
project(pcl_converter)
## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
					roscpp 
					car_msgs
					# geometry_msgs
					pcl_msgs
					pcl_conversions
					visualization_msgs
					pcl_ros 
					#message_generation
					#${msg_dependencies}
)

## Define catkin package
catkin_package()

## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${pcl_msgs_INCLUDE_DIRS})
include_directories(include ${pcl_ros_INCLUDE_DIRS})
include_directories(include ${/include}) 

# Declare executables
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${pcl_msgs_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${pcl_ros_LIBRARIES})

