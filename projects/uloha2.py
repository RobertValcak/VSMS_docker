import pybullet as p
import pybullet_data
import numpy as np
import os
import time

# transformacna DH matica

def dh_transformation(theta, a, d, alpha):
    return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), alpha*np.cos(theta)],
     [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), alpha*np.sin(theta)],
     [0, np.sin(alpha), np.cos(theta), d],
     [0,0,0,1]])

# vykreslenie framu

def draw_frame(T, length=0.2):
    origin = T[:3, 3] # definovanie pociatku
    x_axis = origin + T[:3, 0]*length # definovanie osi
    y_axis = origin + T[:3, 1]*length
    z_axis = origin + T[:3, 2]*length
    
    p.addUserDebugLine(origin, x_axis, [1,0,0], 2)
    p.addUserDebugLine(origin, y_axis, [0,1,0], 2)
    p.addUserDebugLine(origin, z_axis, [0,0,1], 2)
    
if __name__ == "__main__":
    
    physicsClient=p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    robot_id = p.loadURDF("2dof_planar3.urdf", basePosition=[0,0,0], useFixedBase=True) # nacitanie robota
    
    joint_angles = [np.pi/7, np.pi/2] # nastavujeme vlastne uhly
    
    dh_params = [[1,0,1], [1,0,1]] # nastavujeme DH parametre
    
    num_dof = 2 # pocet stupnov volnosti
    
    T = np.eye(4)
    T_list = np.zeros((num_dof, 4, 4))
    for i in range(num_dof): # iterovanie jednotlivych DH prepoctov pre angles
        T_i = dh_transformation(joint_angles[i], *dh_params[i])
        T = T @ T_i
        T_list[i] = T
    
    fk_position_dh = T[:3,3]
    
    for i in range(num_dof):
        p.resetJointState(robot_id, i, joint_angles[i])
        
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240)
        
    ee_state = p.getLinkState(robot_id, 2, computeForwardKinematics=True) # pybullet prepocet FK
    fk_position_pybullet = np.array(ee_state[4])
    
    green_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0,1,0,1]) # vykreslenie guliciek pozicii
    red_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1,0,0,1])
    
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=green_sphere, basePosition=fk_position_dh)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=red_sphere, basePosition=fk_position_pybullet)
    
    prepocet = (fk_position_pybullet - fk_position_dh) # prepocet rozdielu medzi pybullet a DH
    
    print(f"DH Computed Position:  {fk_position_dh}")
    print(f"Pybullet Computed Position:  {fk_position_pybullet}")
    print(f"Rozdiel pozicii:  {prepocet}")
    
    while True:
        p.stepSimulation()
        time.sleep(1/240)
        
    