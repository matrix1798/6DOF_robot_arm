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

X3 = 0.0
Y3 = -100.0
Z3 = 80.0
roll3 = 180.0
pitch3 = 0
yaw3 = 0
P_3 = [X3, Y3, Z3, roll3, pitch3, yaw3]

P1_to_P2 = robot_6DOF.linearInterpolation(P_1,P_2,100)
P2_to_P3 = robot_6DOF.linearInterpolation(P_2,P_3,100)

phi_matrix = np.hstack((P1_to_P2,P2_to_P3))

# ==========================================
# VISUALIZATION (Zoptymalizowany Matplotlib)
# ==========================================
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 1. Konfiguracja stałego widoku (wykonana TYLKO RAZ)
ax.set_xlim([-100, 300])
ax.set_ylim([-200, 200])
ax.set_zlim([0, 450])
ax.set_xlabel('X [mm]')
ax.set_ylabel('Y [mm]')
ax.set_zlabel('Z [mm]')

# 2. Inicjalizacja pustych obiektów (które będziemy aktualizować)
# Wykres ramienia robota:
robot_line, = ax.plot([], [], [], '-o', color='blue', linewidth=4, markersize=5)

# Pole tekstowe:
text_box = ax.text2D(0.05, 0.85, "", transform=ax.transAxes, fontsize=11, 
                     bbox=dict(facecolor='white', alpha=0.8))

for q in phi_matrix.T:
    # Oblicz transformacje
    T_all = robot.fkine_all(q)
    pts = T_all.t 
    
    # 3. Aktualizacja współrzędnych linii (szkielet robota)
    robot_line.set_data(pts[:, 0], pts[:, 1])
    robot_line.set_3d_properties(pts[:, 2])
    
    # 4. Aktualizacja tekstu efektora
    x, y, z = T_all[-1].t
    text_box.set_text(f"Effector:\nX: {x:.3f} mm\nY: {y:.3f} mm\nZ: {z:.3f} mm")
    
    # Wskazówka: trplot jest bardzo wolny do animowania "w locie" w Matplotlib.
    # Jeśli koniecznie potrzebujesz widzieć układy współrzędnych, 
    # zrób to korzystając z wbudowanych narzędzi biblioteki (Opcja 2).
    
    # Odświeżenie widoku (krótszy czas = szybciej)
    plt.pause(0.001)

plt.show()