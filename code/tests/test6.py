import numpy as np
from ..tools.kinematicTools import robot6DOF
import code.tools.mathTools as mt

L_0, L_1, L_2, L_3, L_4, L_5 = 120.925, 54, 93.255, 27, 103.225, 0.0
robot = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

X = 100
Y = 0
Z = 100
roll = 180.0
pitch = 0
yaw = 0

angles = robot.inverseKinematic(X,Y,Z,roll,pitch,yaw)

DH_mat = robot.forwardKinematic(angles)
print(DH_mat)

print(mt.rotation_mat(roll,pitch,yaw))

