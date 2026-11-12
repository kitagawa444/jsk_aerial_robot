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
  roscpp
  nav_msgs
  std_msgs
  std_srvs
  )

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_message_files(FILES GraspReference.msg GraspTrackingStatus.msg)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  aerial_robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bee_robot_model bee_controller bee_navigation bee_cooperative_grasp
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor beetle diagnostic_msgs message_runtime geometry_msgs nav_msgs std_msgs std_srvs
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

add_library(bee_cooperative_grasp
  src/cooperative_grasp/config.cpp
  src/cooperative_grasp/state_estimator.cpp
  src/cooperative_grasp/phase_manager.cpp
  src/cooperative_grasp/controller.cpp
  src/cooperative_grasp/normal_force_control.cpp
  src/cooperative_grasp/vertical_control.cpp
  src/cooperative_grasp/attitude_control.cpp
  src/cooperative_grasp/tracking_reference.cpp
  src/cooperative_grasp/safety_monitor.cpp
)

add_executable(bee_cooperative_grasp_node src/cooperative_grasp/node.cpp)
target_link_libraries(bee_cooperative_grasp_node bee_cooperative_grasp ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(bee_cooperative_grasp_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_executable(bee_grasp_tracker_node src/cooperative_grasp/tracker_node.cpp)
target_link_libraries(bee_grasp_tracker_node bee_cooperative_grasp ${catkin_LIBRARIES})
add_dependencies(bee_grasp_tracker_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_cooperative_grasp test/test_cooperative_grasp.cpp)
  if(TARGET test_cooperative_grasp)
    target_link_libraries(test_cooperative_grasp bee_cooperative_grasp)
    target_compile_definitions(test_cooperative_grasp PRIVATE BEE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  endif()
endif()

install(TARGETS bee_sensor_pluginlib bee_robot_model bee_controller bee_navigation bee_cooperative_grasp bee_cooperative_grasp_node bee_grasp_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/bee/cooperative_grasp
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
