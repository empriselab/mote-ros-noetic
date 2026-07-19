cmake_minimum_required(VERSION 3.0.2)
project(mote_demos)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
