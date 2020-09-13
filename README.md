# SPEAR Turtlebot

## Linux Instructions

Install docker, docker compose, and git if you do not already have them installed.

- Instructions for installing docker: https://docs.docker.com/engine/install/
- Instructions for install docker compose: https://docs.docker.com/compose/install/
- Instructions for installing git: https://git-scm.com/book/en/v2/Getting-Started-Installing-Git

Clone this repository and cd into it:

``` bash
git clone https://github.com/UofA-SPEAR/spear-turtlebot.git
cd spear-turtlebot
```

Build the docker image using docker compose:
```bash
docker-compose build spear-turtlebot
```

Run a docker container:
```bash
docker-compose run spear-turtlebot
```

Inside the docker container, you can launch a gazebo simulation as follows:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
