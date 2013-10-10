cmake_minimum_required(VERSION 2.8.3)
project(ax2550)

find_package(catkin REQUIRED
  COMPONENTS
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
  LIBRARIES ax2550
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp serial serial_utils tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

## ax2550 ROS agnostic library
add_library(ax2550 src/ax2550.cc)
target_link_libraries(ax2550 ${serial_LIBRARIES} ${serial_utils_LIBRARIES} ${Boost_LIBRARIES})

## ax2550 ROS Node
add_executable(ax2550_node src/ax2550_node.cc)
target_link_libraries(ax2550_node ax2550 ${catkin_LIBRARIES})
add_dependencies(ax2550_node ax2550_generate_messages_cpp ${catkin_EXPORTED_LIBRARIES})

install(TARGETS ax2550 ax2550_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
