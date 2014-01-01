cmake_minimum_required(VERSION 2.8.3)
project(redblade_ax2550)

#Added by RJ for threading
set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED
  COMPONENTS
  sensor_msgs
  geometry_msgs
  nav_msgs
  roscpp
  serial
  serial_utils
  tf
  )

find_package(Boost REQUIRED COMPONENTS system thread)

add_message_files(
  FILES
    Encoders.msg
    StampedEncoders.msg
)

generate_messages(DEPENDENCIES geometry_msgs nav_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ax2550 odometry_skid_steer
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp serial serial_utils tf sensor_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

## ax2550 ROS agnostic library
add_library(ax2550 src/ax2550.cc)
add_library(odometry_skid_steer
  src/odometry_skid_steer.cpp
)

target_link_libraries(ax2550 ${serial_LIBRARIES} ${serial_utils_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(odometry_skid_steer ${serial_LIBRARIES} ${serial_utils_LIBRARIES} ${Boost_LIBRARIES})

## ax2550 ROS Node
add_executable(ax2550_node src/ax2550_node.cc)
add_executable(ax2550_awd_node src/ax2550_awd_node.cc)

target_link_libraries(ax2550_node ax2550 ${catkin_LIBRARIES})
target_link_libraries(ax2550_awd_node ${catkin_LIBRARIES} odometry_skid_steer ax2550)

add_dependencies(ax2550_node ax2550_generate_messages_cpp ${catkin_EXPORTED_LIBRARIES})
add_dependencies(ax2550_awd_node ax2550_generate_messages_cpp ${catkin_EXPORTED_LIBRARIES})

install(TARGETS ax2550 odometry_skid_steer ax2550_node ax2550_awd_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
