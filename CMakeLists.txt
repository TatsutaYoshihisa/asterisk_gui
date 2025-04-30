cmake_minimum_required(VERSION 3.0.2)
project(asterisk_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
  message_generation
)

catkin_python_setup()


add_message_files(
  FILES
  LegCommand.msg
  LegPosition.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py std_msgs message_runtime
)

install(PROGRAMS
  scripts/leg_controller_gui.py
  scripts/hexapod_controller_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)