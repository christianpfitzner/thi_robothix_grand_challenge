cmake_minimum_required(VERSION 3.0.2)
project(thi_robothix_grand_challenge_2023)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    std_srvs
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    franka_gripper
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs  franka_gripper # Or other packages containing msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    std_srvs
    rospy
    cv_bridge
    std_msgs
    sensor_msgs
    franka_gripper
    actionlib_msgs
  DEPENDS
    EIGEN3
)

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(pick_and_place_tutorial_lsg        src/pick_and_place_lsg.cpp)
target_link_libraries(pick_and_place_tutorial_lsg ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(press_button src/press_button.cpp)
target_link_libraries(press_button ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(detect_box src/detect_box.cpp)
target_link_libraries(detect_box ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(detect_touch_node src/detect_touch_node.cpp)
target_link_libraries(detect_touch_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(speed_test_node src/speed_test_node.cpp)
target_link_libraries(speed_test_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_install_python(PROGRAMS
    scripts/box_aprox_detection.py
    scripts/read_display.py
    scripts/battery_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(pick_and_place_collision src/pick_and_place_collision.cpp)
# target_link_libraries(pick_and_place_collision ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_executable(pick_and_place_collision_depth_camera src/pick_and_place_collision_depth_camera.cpp)
# target_link_libraries(pick_and_place_collision_depth_camera ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_executable(approach_marker src/approach_marker.cpp)
# target_link_libraries(approach_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(TARGETS pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS pick_and_place_collision DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS pick_and_place_collision_depth_camera DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
