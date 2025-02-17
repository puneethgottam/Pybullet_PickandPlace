import pybullet as p
import pybullet_data
import numpy as np
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Robot_env:

    def __init__(self, urdf_file:str, basePosition, node):

        self.arm = p.loadURDF(urdf_file, useFixedBase = 1, basePosition = basePosition)
        self.ee_link =  8 # Based on urdf file
        self.suction_active = False
        link_state = p.getLinkState(self.arm, self.ee_link)
        self.start_position, self.start_orientation = link_state[0], link_state[1]
        self.cube_pos = [0.6,0,1.26]
        self.cube_orientation = p.getQuaternionFromEuler([0,0,0])
        self.cube =  p.loadURDF('cube_small.urdf', basePosition=self.cube_pos, baseOrientation=self.cube_orientation) #5cm cube default asset
        self.node = node

    def move_to_position(self,target_pos, target_orien):

        joint_positions = p.calculateInverseKinematics(self.arm, self.ee_link, target_pos, target_orien)
        for i in range(len(joint_positions)):
            p.setJointMotorControl2(self.arm, i, p.POSITION_CONTROL, joint_positions[i])
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./10.)
            img = self.attach_camera()
            self.node.publish_image(img)
    
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
            self.node.publish_status('Suction ON')

        else:
            print('Suction is already on')

    def suction_off(self):

        if self.suction_active == True:
            p.removeConstraint(self.suction_constraint)
            self.suction_active = False
            print('Suction is turned off')
            self.node.publish_status('Suction OFF')

        else:
            print('Suction is already off')
        
    def attach_camera(self):
        """ Captures an image from a camera attached to the wrist. Considered the link before end effector gripper as wrist"""
        # Get wrist position and orientation to attach camera
        wrist_state = p.getLinkState(self.arm, self.ee_link-1, computeForwardKinematics=True)
        wrist_pos = np.array(wrist_state[0])  # (x, y, z) position
        wrist_ori = p.getMatrixFromQuaternion(wrist_state[1])  # Rotation matrix

        # Camera Parameters
        cam_target = wrist_pos + np.array([wrist_ori[2], wrist_ori[5], wrist_ori[8]])   # Forward direction
        cam_up = [wrist_ori[1], wrist_ori[4], wrist_ori[7]]  # Camera Up Vector
        cam_pos = wrist_pos + np.matmul(np.array(wrist_ori).reshape(3,3), np.transpose(np.array([0.05,0,0])))

        view_matrix = p.computeViewMatrix(cam_pos, cam_target, cam_up)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.01, farVal=2.0)

        width, height, rgb_img, _, _ = p.getCameraImage(width=320, height=256, viewMatrix=view_matrix, projectionMatrix=proj_matrix)

        return (np.array(rgb_img, dtype=np.uint8))[:,:,:3]  # Slice image from HxWx4 (includes depth) to HxWx3


class PickandDrop:
    
    def __init__(self, robot_env, drop_position, drop_orientation):

        self.env = robot_env
        self.drop_position = drop_position
        self.drop_orientation = drop_orientation

    def execute(self):

        self.env.move_to_position([0.59, 0,1.333], p.getQuaternionFromEuler([0,3.1415,0]))

        if len(p.getContactPoints(self.env.arm, self.env.cube)) :
            self.env.suction_on()
            self.env.node.publish_status("Picked the cube")
            self.env.move_to_position([0.6,0,1.6], self.drop_orientation)
            self.env.move_to_position(self.drop_position, self.drop_orientation)
            self.env.node.publish_status("Placed the cube")
            self.env.suction_off()
            self.env.node.publish_status("Moving back to original position")
            self.env.move_to_position(self.env.start_position, self.env.start_orientation)
            self.env.node.publish_status("Pick and Place task complete")
            print('Pickup and Drop simulation executed')

        else:
            print('No contact between Robot and itme')

class ROS2Node(Node):

    def __init__(self):
        super().__init__('robot_ros2_node')

        # Publishers
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        self.bridge = CvBridge()
        self.get_logger().info("ROS2 Node Initialized.")

    def publish_image(self, image):
        """Publish camera image to ROS2."""
        
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        self.camera_pub.publish(img_msg)

    def publish_status(self, status):
        """Publish robot status."""

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)