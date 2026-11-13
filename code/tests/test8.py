import numpy as np
from ..tools.kinematicTools import robot6DOF

L_0, L_1, L_2, L_3, L_4, L_5 = 120.925, 54, 93.255, 27, 93.225, 11.0
robot6DOF = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

RPM = 60
SPR = 4086
RPS = RPM/60.0
max_SPS = RPS * SPR

max_speed = (np.pi/3.0) / 0.167

fill_of_speed = 1000
speed = fill_of_speed/4096.0 * 2*np.pi

