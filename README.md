# Pybullet_PickandPlace
Project of a robotic arm simulation for pick and place task using Pybullet

# Installation 

Clone the git repository 

```bash
git clone -b main https://github.com/puneethgottam/Pybullet_PickandPlace.git
```
## Docker 

Build the image using Dockerfile. The docker image has ROS2 Humble, Python3 and Pybullet

```bash
docker compose build
docker compose up
```

# Usage

## Pybullet

Run the PickandPlace.py file for simulation. Open new terminal and execute the below commands.

```bash
docker exec -it "container name" bash
source /opt/ros/humble/setup.bash
python3 /workspace/PickandPlace.py
```
## ROS2 

Topic **_/camera/image_raw_** consists of rgb images. 
Topic  **_/robot_status_** consists of robot status during the task.
# Note

## URDF files
The urdf files are obtained from the [ros_industrial/kuka_experimental](https://github.com/ros-industrial/kuka_experimental) repository. The urdf file was edited to add the end effector of this task specification. 

## Docker 
The docker image with ros2 humble was taken from [docker hub] (https://hub.docker.com/r/althack/ros2/tags)
The docker-compose.yml must be edited accordinly for enabling the GPU capabilities. 



