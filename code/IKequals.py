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

X = 0.349
Y = 0.077
Z = 0.427
r = -138.423
p = -6.089
yaw = -66.793

L_0, L_1, L_2, L_3, L_4, L_5 = 0.15, 0.20, 0.15, 0.15, 0.15, 0.05

P_goal = np.array([X,Y,Z,np.radians(r),np.radians(p),np.radians(yaw)])

Rotation_goal = rotation_mat(roll = r, pitch = p, yaw = yaw)
P_wc = P_goal[0:3] - Rotation_goal@np.array([0,0,L_4+L_5]).T

#phi_0
phi_0 = np.atan2(P_wc[1],P_wc[0])

#zmienne pomocnicze
x = P_wc[0]
y = P_wc[1]

R = np.sqrt(x**2 + y**2)
z = P_wc[2] - L_0
L_23 = L_2 + L_3
D = (L_1**2 + L_23**2 - R**2 - z**2)/(2*L_1*L_23)

#phi_2 +/- zalezy od polozenia łokcia
phi_2 = np.atan2(D,-np.sqrt(1-D**2))

#zmienne pomocnicze
k1 = L_1 - L_23*np.sin(phi_2)
k2 = L_23*np.cos(phi_2)

#phi_1
phi_1 = np.atan2(k1*z - k2*R, k1*R + k2*z)

#rotacja 0-3
mat_rot_0_1 = np.array([
        [np.cos(phi_0), 0,  np.sin(phi_0),0],
        [np.sin(phi_0), 0, -np.cos(phi_0),0],
        [  0 , 1,   0 ,L_0],
        [0,0,0,1]
        ])

mat_rot_1_2 = np.array([
        [np.cos(phi_1), -np.sin(phi_1),  0,L_1*np.cos(phi_1)],
        [np.sin(phi_1), np.cos(phi_1), 0, L_1*np.sin(phi_1)],
        [  0 , 0,   1  , 0],
        [0,0,0,1]
        ])

mat_rot_2_3 = np.array([
        [np.cos(phi_2), 0,  -np.sin(phi_2),0],
        [np.sin(phi_2), 0, np.cos(phi_2),0],
        [  0 , -1,   0 , 0],
        [0,0,0,1]
        ])

R_0_3 = mat_rot_0_1@mat_rot_1_2@mat_rot_2_3
R_0_3 = R_0_3[:3,:3]
R_3_6 = R_0_3.T @ Rotation_goal

#phi_3, phi_4 i phi_5 
sin_phi4 = np.sqrt(R_3_6[0, 2]**2 + R_3_6[1, 2]**2)
cos_phi4 = R_3_6[2, 2]

# Jeśli nadgarstek jest wyprostowany (osobliwość)
if sin_phi4 < 1e-6:
    phi_4 = 0.0
    phi_3 = 0.0
    # phi_5 przejmuje całkowity obrót nadgarstka
    phi_5 = np.atan2(R_3_6[1, 0], R_3_6[0, 0])
else:
    # Standardowe wyliczenie
    phi_4 = np.atan2(sin_phi4, cos_phi4)
    phi_3 = np.atan2(-R_3_6[1, 2], -R_3_6[0, 2])
    phi_5 = np.atan2(-R_3_6[2, 1], R_3_6[2, 0])

print(f'\nPozycja celu: x: {X}m, y: {Y}, z: {Z}\npitch: {p}, roll: {r}, yaw: {yaw}')
print(f'Phi_0 = {np.degrees(phi_0)}')
print(f'Phi_1 = {np.degrees(phi_1)}')
print(f'Phi_2 = {np.degrees(phi_2)}')
print(f'Phi_3 = {np.degrees(phi_3)}')
print(f'Phi_4 = {np.degrees(phi_4)}')
print(f'Phi_5 = {np.degrees(phi_5)}')