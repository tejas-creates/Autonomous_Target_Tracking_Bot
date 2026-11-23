cmake_minimum_required(VERSION 2.8.3)

project(simple_assem)

find_package(catkin REQUIRED)
catkin_package()
find_package(roslaunch)

foreach(directory config launch meshes urdf)
  install(DIRECTORY ${directory}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${directory})
endforeach()
