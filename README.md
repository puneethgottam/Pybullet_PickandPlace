# Pybullet_PickandPlace
Project of a robotic arm simulation for pick and place task using Pybullet

# Installation 

Clone the git repository 

'''bash 
git clone -b main https://github.com/puneethgottam/Pybullet_PickandPlace.git
'''

## Docker 

Build the image using Dockerfile 

'''bash
docker compose build
docker compose up
'''

# Usage

Run the PickandPlace.py file for simulation

'''bash
python3 PickandPlace.py
'''

# Note

## URDF files
The urdf files are obtained from the [ros_industrial/kuka_experimental](https://github.com/ros-industrial/kuka_experimental) repository. The urdf file was edited to add the end effector of this task specification. 

## Docker 
The docker image with ros2 humble was taken from [docker hub] (https://hub.docker.com/r/althack/ros2/tags)
The docker-compose.yml must be edited accordinly for enabling the GPU capabilities. 



