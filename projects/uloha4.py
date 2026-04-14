import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time


# parametre
l1 = 1
l2 = 1
link_lengths = [1, 1]

#analyticke riesenia inverznej kinematiky 
def inverse_kinematics(x,y):
    cos_theta2 = ((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
    
    if np.abs(cos_theta2) > 1:
        raise ValueError("Target is out of reach")
    
    solutions = []
    theta2_candidates = [np.acos(cos_theta2), -np.acos(cos_theta2)]
    
    for theta2 in theta2_candidates:
        k1 = l1+l2 * np.cos(theta2)
        k2 = l2*np.sin(theta2)
        theta1 = np.atan2(y,x) - np.atan2(k2,k1)
        solutions.append((theta1, theta2))
    
    return solutions

# dopredna kinematika pre dane uhly
def forward_kinematics_analytical(theta1, theta2):
    x1 = l1*np.cos(theta1)
    y1 = l1*np.sin(theta1)
    x2 = x1 + l2*np.cos(theta1+theta2)
    y2 = y1 + l2*np.sin(theta1+theta2)
    return (0, x1, x2), (0, y1, y2)

# koncova pozicia efektora
def forward_kinematics(theta):
    theta1,theta2 = theta
    x = link_lengths[0] * np.cos(theta1) + link_lengths[1] * np.cos(theta1 + theta2)
    y = link_lengths[0] * np.sin(theta1) + link_lengths[1] * np.sin(theta1 + theta2)
    return np.array([x,y])

# Jacobian 
def jacobian(theta):
    theta1, theta2 = theta
    l1, l2 = link_lengths
    j11 = -l1*np.sin(theta1)-l2*np.sin(theta1+theta2)
    j12 = -l2*np.sin(theta1+theta2)
    j21 = l1*np.cos(theta1)+l2*np.cos(theta1+theta2)
    j22 = l2*np.cos(theta1+theta2)
    return np.array([[j11, j12],[j21,j22]])

# iteracie pomocou Jacobianu 
def inverse_kinematics_numeric(target, theta_init, step=100):
    theta = np.array(theta_init)

    for i in range(step):
        pos= forward_kinematics(theta)
        err = target-pos

        if np.linalg.norm(err)< 1e-3:
            return theta, i

        J = jacobian(theta)
        dtheta = np.linalg.pinv(J) @ err
        theta +=dtheta

    return theta, step

# vzdialenost medzi cielom a dosiahnutou poziciou
def distance_error(target, pos):
    return np.linalg.norm(np.array(target) - np.array(pos))


if __name__=="__main__":

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    analytical_errors = []
    numerical_errors = []
    iterations_list = []

    np.random.seed(0)

    for i in range(10):

        # nahodny bod v dosahu robota
        r = np.random.uniform(0.1, 1.9)
        angle = np.random.uniform(0, 2*np.pi)
        target_pos = [r*np.cos(angle), r*np.sin(angle)]

        print(f"\nTARGET {i+1}: {target_pos}")

        # analyticka IK vypocet a chyba
        try:
            solutions = inverse_kinematics(target_pos[0], target_pos[1])
            theta1, theta2 = solutions[0]

            x_coords, y_coords = forward_kinematics_analytical(theta1, theta2)
            pos_a = [x_coords[-1], y_coords[-1]]

            err_a = distance_error(target_pos, pos_a)

        except:
            err_a = np.nan

        analytical_errors.append(err_a)
        print("Analytical error:", err_a)

        # numericka IK vypocet, chyba a iteracie
        theta_guess = [0.1, 0.1]

        theta_solution, iters = inverse_kinematics_numeric(target_pos, theta_guess)

        pos_n = forward_kinematics(theta_solution)
        err_n = distance_error(target_pos, pos_n)

        numerical_errors.append(err_n)
        iterations_list.append(iters)

        print("Numerical error:", err_n, "iterations:", iters)

        
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        p.loadURDF("plane.urdf")
        robot_id = p.loadURDF("urdf/2dof_planar2.urdf", [0, 0, 0], useFixedBase=True)

        joint_indices= [0, 1]

        p.loadURDF("sphere_small.urdf", [target_pos[0], target_pos[1], 0], globalScaling=0.1)

        for j, angle in enumerate(theta_solution):
            p.resetJointState(robot_id, joint_indices[j], angle)

        for _ in range(120):
            p.stepSimulation()
            time.sleep(1/240)

    # graf porovnania chyb
    x = np.arange(10)

    plt.figure()
    plt.plot(x, analytical_errors, marker='o', label='Analytical IK')
    plt.plot(x, numerical_errors, marker='x', label='Numerical IK')
    plt.xlabel("Index bodu")
    plt.ylabel("Distance error")
    plt.title("Porovnanie chyb IK")
    plt.legend()
    plt.grid()

    # graf poctu iteracii
    plt.figure()
    plt.plot(x, iterations_list, marker='o')
    plt.xlabel("Index bodu")
    plt.ylabel("Pocet iteracii")
    plt.title("Iteracie numerickej IK")
    plt.grid()

    plt.show()