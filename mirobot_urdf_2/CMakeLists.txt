cmake_minimum_required(VERSION 3.5)
project(mirobot_urdf_2)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(serial_driver REQUIRED)

find_package(gazebo REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} 
        ${rmw_implementation_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${lifecycle_msgs_INCLUDE_DIRS}
        ${rcl_lifecycle_INCLUDE_DIRS}
        ${serial_driver_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS} 
        ${SDFormat_INCLUDE_DIRS})

#[[ament_package(
	DEPENDS
  gazebo_ros
	joint_state_publisher
	robot_state_publisher
	rviz2
	serial_driver
  ament_export_dependencies()
  ament_export_include_directories()
  ament_export_libraries()
)]]

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


add_executable(mirobot_write_node src/mirobot_write_node.cpp)
target_include_directories(mirobot_write_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mirobot_write_node PUBLIC c_std_99 cxx_std_17) 
target_link_libraries(mirobot_write_node ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${serial_driver_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

