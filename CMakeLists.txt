cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_kinematics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  IK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS scripts/fk_test scripts/ik_test
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
