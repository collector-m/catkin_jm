cmake_minimum_required(VERSION 2.8.3)
project(can_to_eth)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)



include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(c2e src/c2e.cpp)
target_link_libraries(c2e ${console_bridge_LIBRARIES})


# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS 
      message_generation
          roscpp 
            rospy
            std_msgs
            tf
            geometry_msgs
          sensor_msgs
          nav_msgs
            )

 add_message_files(
   FILES
   can_data.msg

 )
            
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(CATKIN_DEPENDS roscpp
             CATKIN_DEPENDS message_runtime )

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(c2e_node src/c2e_node.cpp)
target_link_libraries(c2e_node c2e ${catkin_LIBRARIES})

add_executable(odom_pub_node src/odom_pub_node.cpp)
target_link_libraries(odom_pub_node  ${catkin_LIBRARIES})

add_executable(decision_node src/decision_node.cpp)
target_link_libraries(decision_node  ${catkin_LIBRARIES})


install(TARGETS c2e c2e_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


