import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from ..tools.kinematicTools import robot6DOF

# length of each robot part
L_0, L_1, L_2, L_3, L_4, L_5 = 120.925, 54, 93.255, 27, 103.225, 0.0

# robot configurates based on DH method
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=L_0,      a=0,   alpha=np.pi/2), 
    rtb.RevoluteDH(d=0,        a=L_1, alpha=0),      
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi/2),       
    rtb.RevoluteDH(d=L_2+L_3,  a=0,   alpha=np.pi/2),
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi/2),
    rtb.RevoluteDH(d=L_4+L_5,  a=0,   alpha=0)
], name="Robot 6DOF")

# calculate angels for each joint
robot_6DOF = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

X1 = 100.0
Y1 = 100.0
Z1 = 100.0
roll1 = 180.0
pitch1 = 0
yaw1 = 0
P_1 = [X1, Y1, Z1, roll1, pitch1, yaw1]

X2 = 100.0
Y2 = 0.0
Z2 = 100.0
roll2 = 180.0
pitch2 = 0
yaw2 = 0
P_2 = [X2, Y2, Z2, roll2, pitch2, yaw2]

X3 = 100.0
Y3 = 0.0
Z3 = 100.0
roll3 = 180.0
pitch3 = 0
yaw3 = 0
P_3 = [X3, Y3, Z3, roll3, pitch3, yaw3]

P1_to_P2 = robot_6DOF.linearInterpolation(P_1,P_2,100)
P2_to_P3 = robot_6DOF.linearInterpolation(P_2,P_3,100)

phi_matrix = np.hstack((P1_to_P2,P2_to_P3))

# ==========================================
# VISUALIZATION (Custom Matplotlib & Spatialmath)
# ==========================================
import matplotlib.pyplot as plt
from spatialmath.base import trplot

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for q in phi_matrix.T:
    ax.cla()  # Czyszczenie klatki
    
    # Oblicz transformacje dla wszystkich członów robota jednocześnie
    T_all = robot.fkine_all(q)
    
    # Pobierz współrzędne X, Y, Z wszystkich przegubów (jako tablica Nx3)
    pts = T_all.t 
    
    # 1. Narysuj strukturę robota (niebieska linia łącząca przeguby)
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-o', color='blue', linewidth=4, markersize=5)
    
    # 2. Narysuj układy współrzędnych dla każdego przegubu i efektora
    # length - długość strzałek osi (w mm, dopasuj do robota)
    for i, T in enumerate(T_all):
        trplot(T.A, ax=ax, length=40.0, width=1.5, frame=str(i))
    
    # 3. Etykieta pozycji efektora (ostatniego elementu)
    x, y, z = T_all[-1].t
    ax.text2D(0.05, 0.85, f"Effector:\nX: {x:.3f} mm\nY: {y:.3f} mm\nZ: {z:.3f} mm", 
              transform=ax.transAxes, fontsize=11, bbox=dict(facecolor='white', alpha=0.8))
    
    # 4. Konfiguracja widoku kamery (stałe granice, aby obraz nie drgał)
    ax.set_xlim([-100, 300])
    ax.set_ylim([-200, 200])
    ax.set_zlim([0, 450])
    
    # Etykiety głównego układu współrzędnych
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')
    
    plt.pause(0.01)

plt.show()