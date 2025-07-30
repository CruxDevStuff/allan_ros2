cmake_minimum_required(VERSION 3.5)
project(allan_ros2)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fopenmp)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include
${EIGEN3_INCLUDE_DIR})

set(DEPS
rclcpp
std_msgs
rosbag2_cpp
rosbag2_storage
sensor_msgs
px4_msgs 
ament_index_cpp
Eigen3
)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_library(${PROJECT_NAME}_lib SHARED 
src/allan_node.cc
src/compute.cc
src/write.cc
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPS})

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(${PROJECT_NAME}_lib "allan_ros::AllanNode")

add_executable(allan_node src/allan_node_main.cc)

target_link_libraries(allan_node ${PROJECT_NAME}_lib)

install(
DIRECTORY include/
DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}_lib 
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS allan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config  
  DESTINATION share/${PROJECT_NAME})

ament_package()
