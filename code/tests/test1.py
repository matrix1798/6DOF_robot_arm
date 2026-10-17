from ..tools.kinematicTools import robot6DOF

"""
    Test of implemented invers kinematic equals of 6DOF robot arm. 
"""

# creat objoect with assignment of a value (values of lenght bettwen joints from final model in cad [mm])
L_0, L_1, L_2, L_3, L_4, L_5 = 120.925, 54, 93.255, 27, 103.225, 0.0
robot = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

# a few examples postions of robot efector
# 1
X = 100
Y = 100
Z = 100
roll = 180.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('1. Streightening arm:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')

X = 100.0
Y = 0.0
Z = 100.0
roll = 180.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('2:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')

X = 0.0
Y = 100.0
Z = 100.0
roll = 180.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('3:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')

X = 0.0
Y = 150.0
Z = 100.0
roll = 180.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('4:')
robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')