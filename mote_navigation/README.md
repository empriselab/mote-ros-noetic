apt-get update
apt-get install -y ros-noetic-gmapping
roslaunch mote_navigation gmapping.launch
rosrun map_server map_saver -f src/mote-ros-noetic/mote_demos/maps/<map_name>