# SPEAR Turtlebot

## Linux Instructions

Install docker and docker-compose.

Build docker image:
```bash
docker-compose build spear-turtlebot
```

Run docker container:
```bash
docker-compose run spear-turtlebot
```

Launch turtlebot in gazebo:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
