# Mote ROS Noetic driver

Determine your robot's IP on the network, then run from within this directory:
```
ROBOT_IP=192.168.x.x docker compose up
```
This pulls the latest published image (`ghcr.io/empriselab/mote-ros-noetic:latest`).
If you've made local changes (e.g. adding your own node), build instead with:
```
ROBOT_IP=192.168.x.x docker compose up --build
```

Open your browser to https://app.foxglove.dev/ and connect to ws://localhost:8765
