import pybullet as p
import pybullet_data
import numpy as np
import time

class Robot_env:

    def __init__(self, urdf_file:str, basePosition= [0,0,0]):

        self.arm = p.loadURDF(urdf_file, useFixedBase = 1, basePosition = basePosition)
        self.ee_link =  8 # Based on urdf file
        self.suction_active = False
        link_state = p.getLinkState(self.arm, self.ee_link)
        self.start_position, self.start_orientation = link_state[0], link_state[1]
        self.cube_pos = [0.6,0,1.26]
        self.cube_orientation = p.getQuaternionFromEuler([0,0,0])
        self.cube =  p.loadURDF('cube_small.urdf', basePosition=self.cube_pos, baseOrientation=self.cube_orientation) #5cm cube default asset

    def move_to_position(self,target_pos, target_orien):
        joint_positions = p.calculateInverseKinematics(self.arm, self.ee_link, target_pos, target_orien)
        for i in range(len(joint_positions)):
            p.setJointMotorControl2(self.arm, i, p.POSITION_CONTROL, joint_positions[i])
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./10.)
            self.attach_camera()
    
    def suction_on(self):
        if self.suction_active == False:
            self.suction_constraint = p.createConstraint(parentBodyUniqueId= self.arm,
                                            parentLinkIndex= self.ee_link,
                                            childBodyUniqueId= self.cube,
                                            childLinkIndex=-1,
                                            jointType=p.JOINT_FIXED,
                                            jointAxis=[0, 0, 0],
                                            parentFramePosition=[0, 0, 0.05],
                                            childFramePosition=[0, 0, 0], 
                                            childFrameOrientation=p.getQuaternionFromEuler([0, np.pi, 0]))
            self.suction_active = True
            print('Suction is turned on')
        else:
            print('Suction is already on')

    def suction_off(self):
        if self.suction_active == True:
            p.removeConstraint(self.suction_constraint)
            self.suction_active = False
            print('Suction is turned off')
        else:
            print('Suction is already off')
        
    def attach_camera(self):
        """ Captures an image from a camera attached to the wrist """
        # Get wrist position and orientation
        wrist_state = p.getLinkState(self.arm, self.ee_link-1, computeForwardKinematics=True)
        wrist_pos = np.array(wrist_state[0])  # (x, y, z) position
        wrist_ori = p.getMatrixFromQuaternion(wrist_state[1])  # Rotation matrix

        # Camera Parameters
        cam_target = wrist_pos + np.array([wrist_ori[2], wrist_ori[5], wrist_ori[8]])   # Forward direction
        cam_up = [wrist_ori[1], wrist_ori[4], wrist_ori[7]]  # Camera Up Vector
        cam_pos = wrist_pos + np.matmul(np.array(wrist_ori).reshape(3,3), np.transpose(np.array([0.05,0,0])))

        view_matrix = p.computeViewMatrix(cam_pos, cam_target, cam_up)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.01, farVal=2.0)

        width, height, rgb_img, _, _ = p.getCameraImage(
            width=256, height=256, viewMatrix=view_matrix, projectionMatrix=proj_matrix
        )

        return np.reshape(rgb_img, (height, width, 4))  # Reshape image


class PickandDrop:
    
    def __init__(self, robot_env, drop_position, drop_orientation):

        self.env = robot_env
        self.drop_position = drop_position
        self.drop_orientation = drop_orientation

    def execute(self):

        self.env.move_to_position([0.59, 0,1.333], p.getQuaternionFromEuler([0,3.1415,0]))

        contacts = p.getContactPoints(self.env.arm, self.env.cube)
        if len(contacts) :

            self.env.suction_on()
            self.env.move_to_position([0.6,0,1.6], self.drop_orientation)
            self.env.move_to_position(self.drop_position, self.drop_orientation)
            self.env.suction_off()
            self.env.move_to_position(self.env.start_position, self.env.start_orientation)

            print('Pickup and Drop simulation executed')

        else:
            print('No contact between Robot and itme')