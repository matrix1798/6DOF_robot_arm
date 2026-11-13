import numpy as np
import matplotlib.pyplot as plt
from ..tools.kinematicTools import robot6DOF

L_0, L_1, L_2, L_3, L_4, L_5 = 120.925, 54, 93.255, 27, 93.225, 11.0
robot6DOF = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

X2, Y2, Z2 = 160, 0, 80
roll2, pitch2, yaw2 = 180, 0, 0
P_2 = [X2, Y2, Z2, roll2, pitch2, yaw2]

X3, Y3, Z3 = 0, 160, 0
roll3, pitch3, yaw3 = 180, 0, 180
P_3 = [X3, Y3, Z3, roll3, pitch3, yaw3]

q_matrix = robot6DOF.linearInterpolation(P_2, P_3, 100)

# 2. Ekstrakcja współrzędnych X, Y, Z za pomocą kinematyki prostej
x_traj = []
y_traj = []
z_traj = []

# Iterujemy po kolumnach macierzy (po poszczególnych krokach czasowych/interpolacyjnych)
for i in range(q_matrix.shape[1]):
  angles = q_matrix[:, i]
  X, Y, Z, roll, pitch, yaw = robot6DOF.forwardKinematic(angles)
  x_traj.append(X)
  y_traj.append(Y)
  z_traj.append(Z)

# 3. Rysowanie wykresu 3D
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Linia trajektorii
ax.plot(
    x_traj,
    y_traj,
    z_traj,
    label='Trajektoria efektora',
    color='blue',
    linewidth=2,
)

# Punkt początkowy i końcowy
ax.scatter(
    [x_traj[0]],
    [y_traj[0]],
    [z_traj[0]],
    color='green',
    s=60,
    label='Początek (Start)',
)
ax.scatter(
    [x_traj[-1]],
    [y_traj[-1]],
    [z_traj[-1]],
    color='red',
    s=60,
    label='Koniec (Goal)',
)

# Etykiety osi i tytuł
ax.set_xlabel('Oś X [m]')
ax.set_ylabel('Oś Y [m]')
ax.set_zlabel('Oś Z [m]')
ax.set_title('Trajektoria ruchu efektora 6DOF')
ax.legend()

# Wyświetlenie wykresu
plt.show()