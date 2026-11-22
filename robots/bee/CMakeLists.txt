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
  sensor_msgs
  smach_msgs
  visualization_msgs
  roscpp
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  urdf
  )

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Threads REQUIRED)

add_message_files(FILES BeeCommand.msg ControllerStatus.msg ForceFeedback.msg ObserverCalibrationStatus.msg ForceControlParameters.msg)
add_service_files(FILES SetForceControlParameters.srv)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  aerial_robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bee_robot_model bee_controller bee_navigation bee_cooperative_grasp
  DEPENDS OsqpEigen
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor beetle diagnostic_msgs message_runtime geometry_msgs nav_msgs std_msgs std_srvs tf2_ros urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(bee_controller src/control/bee_controller.cpp)
target_link_libraries(bee_controller bee_robot_model bee_navigation ${catkin_LIBRARIES})
add_dependencies(bee_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(bee_robot_model src/model/bee_robot_model.cpp)
target_link_libraries(bee_robot_model ${catkin_LIBRARIES})

add_library(bee_navigation src/bee_navigation.cpp)
target_link_libraries(bee_navigation bee_robot_model ${catkin_LIBRARIES})

add_library(bee_cooperative_grasp
  src/cooperative_grasp/grasp_state_estimator.cpp
  src/cooperative_grasp/phase_manager.cpp
  src/cooperative_grasp/grasp_controller.cpp
  src/cooperative_grasp/wrench_allocator.cpp
)

target_link_libraries(bee_cooperative_grasp PUBLIC OsqpEigen::OsqpEigen)

add_executable(bee_grasp_joy src/grasp_joy.cpp)
target_link_libraries(bee_grasp_joy ${catkin_LIBRARIES})

add_executable(bee_cooperative_grasp_node src/cooperative_grasp/grasp_node.cpp)
target_link_libraries(bee_cooperative_grasp_node bee_cooperative_grasp bee_robot_model ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(bee_cooperative_grasp_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  add_executable(reference_tracker_harness EXCLUDE_FROM_ALL test/reference_tracker_harness.cpp)
  target_link_libraries(reference_tracker_harness bee_controller ${catkin_LIBRARIES})
  add_dependencies(reference_tracker_harness ${${PROJECT_NAME}_EXPORTED_TARGETS})
  catkin_add_gtest(test_cooperative_grasp test/test_cooperative_grasp.cpp)
  if(TARGET test_cooperative_grasp)
    target_link_libraries(test_cooperative_grasp bee_cooperative_grasp bee_controller bee_robot_model ${catkin_LIBRARIES})
    add_dependencies(test_cooperative_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_compile_definitions(test_cooperative_grasp PRIVATE BEE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  endif()
endif()

install(TARGETS bee_grasp_joy bee_robot_model bee_controller bee_navigation bee_cooperative_grasp bee_cooperative_grasp_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/bee/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config plugins launch robots urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
