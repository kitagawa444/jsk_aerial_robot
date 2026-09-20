add_rostest(flight_control.test ARGS headless:=true airframe:=bi)
add_rostest(flight_control.test ARGS headless:=true airframe:=tri)
add_rostest(flight_control.test ARGS headless:=true airframe:=tri_omni)
add_rostest(flight_control.test ARGS headless:=true airframe:=quad)

catkin_add_gtest(gimbalrotor_yaw_scale_test yaw_scale_test.cpp)
if(TARGET gimbalrotor_yaw_scale_test)
  target_link_libraries(gimbalrotor_yaw_scale_test gimbalrotor_controller gimbalrotor_robot_model ${catkin_LIBRARIES})
endif()
