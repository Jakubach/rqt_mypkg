cmake_minimum_required(VERSION 3.0.2)
project(rqt_mypkg)
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)
catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS 
    rospy 
    rqt_gui
    rqt_gui_py
)

catkin_install_python(PROGRAMS scripts/ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
    resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

