from ..tools.kinematicTools import robot6DOF

"""
    Test of coversion to real angles in robot arm
"""

# creat objoect with assignment of a value (values of lenght bettwen joints from final model in cad [mm])
L_0, L_1, L_2, L_3, L_4, L_5 = 120.925, 54, 93.255, 27, 103.225, 0.0

limits_map = {
    'min_angle': [ 40, 90, 60,  0, 25,0],
    'max_angle': [320,290,300,360,280,0],
    'offset':    [ -4,  2,  0,-48,-27,0]
}

robot = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

# a few examples postions of robot efector
# 1
X = 120
Y = 0
Z = 80
roll = 180.0
pitch = 0.0
yaw = 0.0
print(40*'=')
print('1. Streightening arm:')
angles = robot.inverseKinematic(X, Y, Z, roll, pitch, yaw, elbow_pos='up')
print(robot.angleConvert(angles))
print("\n")


