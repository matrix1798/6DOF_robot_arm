import numpy as np
import matplotlib.pyplot as plt
import kinematicTools as KT

# Generowanie macierzy transformacji ze standardowych parametrów DH
def dh_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

# Twoje parametry
L_0, L_1, L_2, L_3, L_4, L_5 = 0.15, 0.20, 0.15, 0.15, 0.15, 0.05

# Przykładowe 6 kątów (q1..q6) do przetestowania (w radianach)

X = 0.2
Y = 0.2
Z = 0.1
roll = 0.0
pitch = 180.0
yaw = 0.0

q = KT.inverseKinematic(X,Y,Z,roll,pitch,yaw)

# Tabela DH w kolejności: [theta, d, a, alpha]
dh_params = [
    [q[0], L_0,             0,    np.pi/2],
    [q[1], 0,               L_1,  0],
    [q[2], 0,               0,   -np.pi/2],
    [q[3], L_2 + L_3,       0,    np.pi/2],
    [q[4], 0,               0,   -np.pi/2],
    [q[5], L_4 + L_5,       0,    0]
]

# Obliczanie pozycji (kinematyka prosta)
points = [np.array([0, 0, 0])] # Start bazy
T = np.eye(4)

for params in dh_params:
    T = T @ dh_matrix(*params)
    points.append(T[:3, 3])

points = np.array(points)
xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]

# Rysowanie wykresu 3D
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Rysowanie linii robota i węzłów
ax.plot(xs, ys, zs, marker='o', linewidth=4, markersize=8, color='b')

# Formatowanie osi (skala dostosowana do długości członów)
lim = 0.5
ax.set_xlim([-lim, lim])
ax.set_ylim([-lim, lim])
ax.set_zlim([0, lim])
ax.set_xlabel('Oś X [m]')
ax.set_ylabel('Oś Y [m]')
ax.set_zlabel('Oś Z [m]')
ax.set_title(f'Robot DH 3D\nPozycja efektora: X={xs[-1]:.3f}, Y={ys[-1]:.3f}, Z={zs[-1]:.3f}')

plt.show()