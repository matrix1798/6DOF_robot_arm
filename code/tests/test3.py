import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot
from kinematicTools import robot6DOF

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
], name="Robot 6DOF")

#calculate angels for each joint
robot_6DOF = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

X1 = 0.2
Y1 = 0.0
Z1 = 0.3
roll1 = 180
pitch1 = 0
yaw1 = 0
P_1 = [X1, Y1, Z1, roll1, pitch1, yaw1]

X2 = 0.2
Y2 = 0.2
Z2 = 0.3
roll2 = 180
pitch2 = 0
yaw2 = 0
P_2 = [X2, Y2, Z2, roll2, pitch2, yaw2]

X3 = 0.0
Y3 = 0.2
Z3 = 0.3
roll3 = 180
pitch3 = 0
yaw3 = 0
P_3 = [X3, Y3, Z3, roll3, pitch3, yaw3]

P1_to_P2 = robot_6DOF.linearInterpolation(P_1,P_2,100)
P2_to_P3 = robot_6DOF.linearInterpolation(P_2,P_3,100)

phi_matrix = np.hstack((P1_to_P2,P2_to_P3))

#visualization
env = PyPlot()
env.launch()
env.add(robot)

robot_pos_label = env.ax.text2D(0.05, 0.85, "", transform=env.ax.transAxes, 
                              fontsize=11, bbox=dict(facecolor='white', alpha=0.8))

for q in phi_matrix.T:
    robot.q = q
    env.step(0.05)

    T = robot.fkine(q)
    x, y, z = T.t

    robot_pos_label.set_text(f"Effector:\nX: {x:.3f} m\nY: {y:.3f} m\nZ: {z:.3f} m")

env.hold()