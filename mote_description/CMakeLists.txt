cmake_minimum_required(VERSION 3.0.2)
project(mote_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY description launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
