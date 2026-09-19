cmake_minimum_required(VERSION 3.0.2)
project(amr_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package()

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

