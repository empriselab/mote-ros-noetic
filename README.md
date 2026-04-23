# Mote ROS Noetic driver

Determine your robot's IP on the network, then run from within this directory:
```
docker build -t mote-ros-noetic .
ROBOT_IP=192.168.x.x docker compose up
```

Open your browser to https://app.foxglove.dev/ and connect to ws://localhost:8765
