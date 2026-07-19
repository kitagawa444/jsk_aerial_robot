cmake_minimum_required(VERSION 3.0.2)
project(bee)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  gimbalrotor
  beetle
  diagnostic_msgs
  message_generation
  geometry_msgs
  )

find_package(Eigen3 REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bee_robot_model bee_controller bee_navigation
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor beetle diagnostic_msgs message_runtime geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(bee_sensor_pluginlib  src/sensor/imu.cpp)
target_link_libraries(bee_sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(bee_sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

add_library(bee_controller src/control/bee_controller.cpp)
target_link_libraries(bee_controller bee_sensor_pluginlib bee_robot_model ${catkin_LIBRARIES})
add_dependencies(bee_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(bee_robot_model src/model/bee_robot_model.cpp)
target_link_libraries(bee_robot_model ${catkin_LIBRARIES})

add_library(bee_navigation src/bee_navigation.cpp)
target_link_libraries(bee_navigation bee_robot_model ${catkin_LIBRARIES})

install(TARGETS bee_sensor_pluginlib bee_robot_model bee_controller bee_navigation
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)