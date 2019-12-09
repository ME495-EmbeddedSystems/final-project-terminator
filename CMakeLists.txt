cmake_minimum_required(VERSION 2.8.3)
project(terminator)


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
)

catkin_python_setup()


if(CATKIN_ENABLE_TESTING)
find_package(rostest REQUIRED)
add_rostest(test/myTest.test)
endif()


# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES
   AimGun.action
   Commands.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES terminator
   CATKIN_DEPENDS
     actionlib_msgs
     message_generation
     message_runtime
     rospy
     std_msgs
#  DEPENDS system_lib
)


#############
## Install ##
#############


install(PROGRAMS
   nodes/aim_target
   nodes/image_pipeline
   nodes/move_left
   nodes/move_right

   nodes/commander
   nodes/move_left_actiontest
   nodes/move_right_actiontest

#   nodes/mastercommander
   nodes/final_pose
   nodes/finalpose_test
#   nodes/lefttest2
#   nodes/righttest2

   #src/terminator/gripper.py

   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_terminator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
