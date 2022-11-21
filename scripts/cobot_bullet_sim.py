#! /usr/bin/env python3

import rospy
import pybullet as pyb
import pybullet_data
from humanoid_bullet_control import HumanoidControl
from cobot_utils.ur_bullet_control import URControl
from cobot_utils.ur_execution_manager import URExecutionManager
from cobot_utils.human_pose_data_streamer import HumanPoseDataStreamer
import numpy as np

import time

def main():
    # ros node initialization
    rospy.init_node("pybullet_ros_bridge", anonymous=True)
    
    # pybullet initialization
    gui_id = pyb.connect(pyb.GUI)
    pyb.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=gui_id)
    pyb.setGravity(0, 0, -10)
    pyb.setRealTimeSimulation(True)

    # load ground URDF
    ground_id = pyb.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True, physicsClientId=gui_id)

    # load ur_controller
    ur_control = URControl(gui_id)

    # humanoid controller
    humanoid_control = HumanoidControl(gui_id)
    humanoid_control.load_config('/home/omkar/ros_workspaces/robotics_lab_ws/src/cobot_bullet_sim/config/humanoid_controllers.yaml')

    # human pose data loader
    pose_stream = HumanPoseDataStreamer('/home/omkar/ros_workspaces/robotics_lab_ws/src/cobot_bullet_sim/data/hammer5.npy', fps=15)
    pose_stream.start()

    ur_mgr = URExecutionManager()
    
    time.sleep(1)

    # load models
    ur_control.load_model(
        '/home/omkar/ros_workspaces/robotics_lab_ws/src/fmauch_universal_robot/ur_description/urdf/model.urdf',
        [0.5, -0.2, 0.5]
    )
    humanoid_control.load_model('humanoid/humanoid.urdf', pose_stream.get_joint_position_set())

    while not rospy.is_shutdown():
        ur_control.update(ur_mgr.get_joint_state())
        humanoid_control.update(pose_stream.get_joint_position_set())

        collision_info = pyb.getClosestPoints(humanoid_control._humanoid_id, ur_control._ur_id, 0.05)
        if len(collision_info) > 0:
            ur_mgr.cancel_execution()

if __name__ == '__main__':
    main()