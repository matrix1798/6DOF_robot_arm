import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot

#length of each robot part
L_0, L_1, L_2, L_3, L_4, L_5 = 0.15, 0.20, 0.15, 0.15, 0.15, 0.05

#robot configurates based on DH method
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=L_0,      a=0,   alpha=np.pi/2), 
    rtb.RevoluteDH(d=0,        a=L_1, alpha=0),      
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi/2),       
    rtb.RevoluteDH(d=L_2+L_3,  a=0,   alpha=np.pi/2),
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi/2),
    rtb.RevoluteDH(d=L_4+L_5,  a=0,   alpha=0)
], name="Robot ^DOF")

#calculate angels for each joint
q_start = np.array([0, 0, 0, 0, 0, 0])
q_stop = np.array([np.pi/2, np.pi/4, -np.pi/3, np.pi/4, np.pi/2, np.pi])
steps = 100
phi_matrix = np.zeros((steps, 6))
for i in range(6):
    phi_matrix[:, i] = np.linspace(q_start[i], q_stop[i], steps)

#visualization
env = PyPlot()
env.launch()
env.add(robot)

robot_pos_label = env.ax.text2D(0.05, 0.85, "", transform=env.ax.transAxes, 
                              fontsize=11, bbox=dict(facecolor='white', alpha=0.8))

for q in phi_matrix:
    robot.q = q
    env.step(0.05)

    T = robot.fkine(q)
    x, y, z = T.t

    robot_pos_label.set_text(f"Effector:\nX: {x:.3f} m\nY: {y:.3f} m\nZ: {z:.3f} m")

env.hold()