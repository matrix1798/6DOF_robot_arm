import sympy as sp
from tools import rotation_mat
import numpy as np

phi0, phi1, phi2, phi3, phi4, phi5 = sp.symbols('phi_0 phi_1 phi_2 phi_3 phi_4 phi_5')
L_0,L_1,L_2,L_3,L_4,L_5 = sp.symbols('L_0 L_1 L_2 L_3 L_4 L_5')

cos0, sin0 = sp.cos(phi0), sp.sin(phi0)
cos1, sin1 = sp.cos(phi1), sp.sin(phi1)
cos2, sin2 = sp.cos(phi2), sp.sin(phi2)
cos3, sin3 = sp.cos(phi3), sp.sin(phi3)
cos4, sin4 = sp.cos(phi4), sp.sin(phi4)
cos5, sin5 = sp.cos(phi5), sp.sin(phi5)

mat_rot_0_1 = sp.Matrix([
        [cos0, 0,  sin0,0],
        [sin0, 0, -cos0,0],
        [  0 , 1,   0 ,L_0],
        [0,0,0,1]
        ])

mat_rot_1_2 = sp.Matrix([
        [cos1, -sin1,  0,L_1*cos1],
        [sin1, cos1, 0, L_1*sin1],
        [  0 , 0,   1  , 0],
        [0,0,0,1]
        ])

mat_rot_2_3 = sp.Matrix([
        [cos2, 0,  -sin2,0],
        [sin2, 0, cos2,0],
        [  0 , -1,   0 , 0],
        [0,0,0,1]
        ])

mat_rot_3_4 = sp.Matrix([
        [cos3, 0,  sin3, 0],
        [sin3, 0, -cos3, 0],
        [  0 , 1,   0  , L_2+L_3],
        [0,0,0,1]
        ])

mat_rot_4_5 = sp.Matrix([
        [cos4, 0,  -sin4,0],
        [sin4, 0, cos4,0],
        [  0 , -1,   0 ,0 ],
        [0,0,0,1]
        ])

mat_rot_5_6 = sp.Matrix([
        [cos5, -sin5,  0,0],
        [sin5, cos5, 0,0],
        [  0 , 0,   1 , L_4+L_5],
        [0,0,0,1]
        ])

mat_rot_0_3 = mat_rot_0_1 * mat_rot_1_2 * mat_rot_2_3
mat_rot_0_4 = mat_rot_0_1 * mat_rot_1_2 * mat_rot_2_3 * mat_rot_3_4
mat_rot_3_6 = mat_rot_3_4 * mat_rot_4_5 * mat_rot_5_6
mat_rot_4_6 = mat_rot_4_5 * mat_rot_5_6
mat_rot_0_6 = mat_rot_0_1 * mat_rot_1_2 * mat_rot_2_3 * mat_rot_3_4 * mat_rot_4_5 * mat_rot_5_6

#IK dla 0-4 złacz:
x_equal_0_4 = mat_rot_0_4[0,3]
y_equal_0_4 = mat_rot_0_4[1,3]
z_equal_0_4 = mat_rot_0_4[2,3]

#IK dla 4-6 złacz:
x_equal_4_6 = mat_rot_4_6[0,3]
y_equal_4_6 = mat_rot_4_6[1,3]
z_equal_4_6 = mat_rot_4_6[2,3]

print('IK dla 0-4 złacz:')
print('x = ',end='')
sp.pprint(x_equal_0_4)
print('y = ',end='')
sp.pprint(y_equal_0_4)
print('z = ',end='')
sp.pprint(z_equal_0_4)

print('IK dla 4-6 złacz:')
print('x = ',end='')
sp.pprint(x_equal_4_6)
print('y = ',end='')
sp.pprint(y_equal_4_6)
print('z = ',end='')
sp.pprint(z_equal_4_6)