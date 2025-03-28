from robot_utils import Robot_env, PickandDrop

import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet in GUI mode
p.connect(p.GUI)

# Set gravity to 9.8
p.setGravity(0, 0, -9.8)

## Building environment 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeID = p.loadURDF("plane.urdf") # table and plane are default assets from pybullet

table_id = p.loadURDF("table/table.urdf", basePosition=[0, 0, 0], globalScaling=2)

tray = p.loadURDF('tray/tray.urdf', basePosition = [0.5,0,1.25])

kuka = Robot_env("kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf", basePosition= [0, 0, 1.25])

PandP = PickandDrop(kuka, [0, 0.6, 1.34], p.getQuaternionFromEuler([0, np.pi, 0]))

PandP.execute()

p.disconnect()