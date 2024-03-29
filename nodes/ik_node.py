#!/usr/bin/python3

from mujoco_py import MjSim, MjViewer, load_model_from_path
from dm_control import mujoco
import inverse_kinematics
import numpy as np
import rospy
from geometry_msgs.msg import Point
import time

class IK:
    def __init__(self):
        self.model = load_model_from_path("/home/kallrax/Robotics/MedicalRobotics/dvrk_ws/src/dvrk_psm/mgcf/psm.xml")
        self.sim = MjSim(self.model)
        self.viewer = MjViewer(self.sim)
        
        self.physics = mujoco.Physics.from_xml_path("/home/kallrax/Robotics/MedicalRobotics/dvrk_ws/src/dvrk_psm/mgcf/psm.xml")
        self.site = "endeff"
        self.tol=1e-14
        
        # Subscribers
        self.endeff_next_pos_sub = rospy.Subscriber("/psm/endeff_next_pos", Point, self.inverse_kinematics)
        
        self.freq = 50
        while True:
            time.sleep(1/self.freq)
            self.sim.forward()
            self.viewer.render()
        
    
    def inverse_kinematics(self,data: Point):
        self.target_pos = [data.x,data.y,data.z]
        self.target_quat = None
        qpos = inverse_kinematics.qpos_from_site_pose(physics=self.physics,site_name=self.site,target_pos=self.target_pos,target_quat=self.target_quat,tol=self.tol)[0]
        self.sim.data.qpos[:] = qpos
        
        
def main():
    rospy.init_node('ik_node')
    IK()

if __name__ == "__main__":
    main()
      
        
        
    
    