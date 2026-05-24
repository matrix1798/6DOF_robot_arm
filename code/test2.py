import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import kinematicTools as KT
from kinematicTools import robot6DOF
import mathTools as mt

matplotlib.use('Qt5Agg')

L_0, L_1, L_2, L_3, L_4, L_5 = 0.15, 0.20, 0.15, 0.15, 0.15, 0.05

X = 0.0
Y = 0.4
Z = 0.1
roll = 0.0
pitch = 180.0
yaw = 0.0

robot = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)
phi_start = robot.inverseKinematic(0.0,0.4,0.1,0.0,180.0,0.0)
phi_stop = robot.inverseKinematic(0.0,0.4,0.3,0.0,180.0,0.0)

phi_matrix = robot6DOF.axial_interpolation(start_phi = phi_start, stop_phi = phi_stop, accuracy=50)

#wizulazacja ramienia
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111,projection='3d')

lim = 0.8
ax.set_xlim([-lim, lim])
ax.set_ylim([-lim, lim])
ax.set_zlim([0, lim])

ax.set_xlabel('Oś X [m]')
ax.set_ylabel('Oś Y [m]')
ax.set_zlabel('Oś Z [m]')

arm_position, = ax.plot([],[],[],marker='o',linewidth = 4, markersize = 8, color = 'k')

def update(frame):

    phi_list = phi_matrix[frame,:]

    dh_params = [
        [phi_list[0], L_0,             0,    np.pi/2],
        [phi_list[1], 0,               L_1,  0],
        [phi_list[2], 0,               0,   -np.pi/2],
        [phi_list[3], L_2 + L_3,       0,    np.pi/2],
        [phi_list[4], 0,               0,   -np.pi/2],
        [phi_list[5], L_4 + L_5,       0,    0]
    ]

    points = [np.array([0,0,0])] #punkty startowe

    T = np.eye(4)
    for params in dh_params:
        T = T @ mt.DHmatrix(*params)
        points.append(T[:3,3])

    points = np.array(points)
    xs, ys, zs = points[:,0], points[:,1], points[:,2]

    ax.set_title(f'Robot DH 3D\nPozycja efektora: X={xs[-1]:.3f}, Y={ys[-1]:.3f}, Z={zs[-1]:.3f}')

    arm_position.set_data(xs,ys)
    arm_position.set_3d_properties(zs)
    
    return arm_position

animation = FuncAnimation(fig,update, frames=50, interval = 10, blit = False)
plt.show()