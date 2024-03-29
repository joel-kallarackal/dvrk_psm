from mujoco_py import MjSim, MjViewer, load_model_from_path
import time
from dm_control import mujoco
import ik
import numpy as np
        
def main():
    
    model = load_model_from_path("/home/kallrax/Robotics/MedicalRobotics/dvrk_ws/src/dvrk_psm/mgcf/psm.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)

    freq = 10
    z=0.65
    while True:
        time.sleep(1/freq)
        z=z-0.005
        sim.data.qpos[:] = inverse_kinematics([-0.25,0,z])
        sim.forward()
        viewer.render()
    
def inverse_kinematics(pos = [-0.25,0,0.65]):
    physics = mujoco.Physics.from_xml_path("/home/kallrax/Robotics/MedicalRobotics/dvrk_ws/src/dvrk_psm/mgcf/psm.xml")
    
    site = "endeff"
    target_pos = np.array(pos)
    target_quat = None
    tol=1e-14
    
    return ik.qpos_from_site_pose(physics=physics,site_name=site,target_pos=target_pos,target_quat=target_quat,tol=tol)[0]
    
       
 
    
if __name__ == "__main__":
     main()