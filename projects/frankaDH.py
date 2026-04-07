import pybullet as p
import pybullet_data
import numpy as np
import os
import time
 
 
def dh_transformation(a,d,theta,alpha):
    return np.array([[np.cos(theta), -np.sin(theta), 0 , np.pi/2],
     [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
     [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
     [0,0,0,1]])
    
    
    
if __name__ == "__main__":
    
    physicsClient=p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    robot_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0,0,0], useFixedBase=True)
    
    joint_angles = [0, -np.pi/4, np.pi/4, -np.pi/2, np.pi/2, np.pi/2, np.pi/3]
    
    dh_params = [
        [0, 0.333, 0],
        [0, 0, -np.pi/2],
        [0, 0.316, np.pi/2],
        [0.0825, 0, np.pi/2],
        [-0.0825, 0.384, -np.pi/2],
        [0, 0, np.pi/2],
        [0.088, 0.107, np.pi/2]
        
    ]
    
    num_dof = 7
    
    T = np.eye(4)
    T_list = np.zeros((num_dof, 4, 4))
    for i in range(num_dof):
        T_i = dh_transformation(joint_angles[i], *dh_params[i])
        T = T @ T_i
        T_list[i] = T
        
    fk_position_dh = T[:3,3]
    
    for i in range(7):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])
        
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240)
        
    ee_state = p.getLinkState(robot_id, 8, computeForwardKinematics=True)
    fk_position_pybullet = np.array(ee_state[4])
        
    sphere_visual_shape = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[0,0,1,1])
    
    for i in range(num_dof):
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_visual_shape, basePosition=T_list[i, :3, 3])
 
    print(T_list[3, :3, 3],)
    
    p.addUserDebugPoints(T_list[:, :3, 3], [[0,0,1]]*num_dof, pointSize=20)
    
    print(f"DH Computed Position: {fk_position_dh}")
    print(f"Pybullet Computed Position: {fk_position_pybullet}")
    
    while True:
        p.stepSimulation()
        time.sleep(1/240)