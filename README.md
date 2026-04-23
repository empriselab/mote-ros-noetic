# Mote ROS Noetic driver

Clone this repo into a folder next to mote-core. This is required for the node to build libmote_ffi.a, which it uses to communicate with the robot.

Next, open the configuration page, determine your robots IP on the network, and fill it into the following command.
```
docker build -f mote-ros-noetic/Dockerfile -t mote-ros-noetic .
ROBOT_IP=192.168.x.x docker compose -f mote-ros-noetic/docker-compose.yml up
```

Open your browser to https://app.foxglove.dev/ and connect to ws://localhost:8765
