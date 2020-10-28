cmake_minimum_required(VERSION 2.8.3)
project(lla2enu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geodesy sensor_msgs tf nav_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg  
)

add_service_files(
  FILES
  ComputeDistance.srv
)

add_message_files(
 FILES
 CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sub_gen src/sub_gen.cpp)
add_dependencies(sub_gen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_gen ${catkin_LIBRARIES})

add_executable(compute_distance src/compute_distance.cpp)
target_link_libraries(compute_distance ${catkin_LIBRARIES})
add_dependencies(compute_distance ${catkin_EXPORTED_TARGETS})

add_executable(client_custom src/client_custom.cpp)
target_link_libraries(client_custom ${catkin_LIBRARIES})
add_dependencies(client_custom ${catkin_EXPORTED_TARGETS})

