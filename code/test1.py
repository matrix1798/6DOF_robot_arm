from kinematicTools import robot6DOF

"""
    Test of implemented invers kinematic equals of 6DOF robot arm. 
"""

# creat objoect with assignment of a value
L_0, L_1, L_2, L_3, L_4, L_5 = 0.15, 0.20, 0.15, 0.15, 0.15, 0.05
robot = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

# a few examples postions of robot efector
# 1
X = 0.0
Y = 0.0
Z = 0.85
roll = 0.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('1. Streightening arm:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')

X = 0.0
Y = 0.0
Z = 0.85
roll = 0.0
pitch = 0.0
yaw = -180.0
print(40*'=')
print('2:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')

X = 0.2
Y = 0.2
Z = 0.2
roll = 180.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('3:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')

X = 0.0
Y = 0.0
Z = 0.85
roll = 0.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('4:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')