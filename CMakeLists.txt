cmake_minimum_required (VERSION 3.0.2)
project (aerial_image_stitching_pkg)

find_package (catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    message_generation
)

find_package (gazebo REQUIRED)

add_message_files (
  FILES
  ImagePose.msg
)

catkin_python_setup()

generate_messages (
  DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs
)

catkin_package (
    CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs message_generation
)

