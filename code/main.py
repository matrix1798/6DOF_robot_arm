import numpy as np

np.set_printoptions(precision=4, suppress=True)

"""
Macierz Denavita-Hartenberga(DH):

i-1_H_i = np.array([
                    [cos(phi_i), -sin(phi_i)*cos(theta_i), sin(phi_i)*sin(theta_i) , gammma_i*cos(phi_i)]
                    [sin(phi_i),  cos(phi_i)*cos(theta_i), -cos(phi_i)*sin(theta_i), gammma_i*sin(phi_i)]
                    [     0    ,        sin(theta_i)     ,        cos(theta_i)     ,       delta_i      ]
                    [     0    ,             0           ,             0           ,          1         ]
                    ])
"""

def DHmatrix (phi_i,delta_i,lambda_i,theta_i):

    return np.array([
              [ np.cos(phi_i), -np.sin(phi_i)*np.cos(theta_i) , np.sin(phi_i)*np.sin(theta_i) , lambda_i*np.cos(phi_i)],
              [ np.sin(phi_i), np.cos(phi_i)*np.cos(theta_i)  , -np.cos(phi_i)*np.sin(theta_i), lambda_i*np.sin(phi_i)],
              [     0        ,          np.sin(theta_i)       ,           np.cos(theta_i)     ,      delta_i          ],
              [     0        ,              0                 ,                 0             ,         1             ]
              ])

theta_1 = 0
theta_2 = np.pi
theta_3 = 0

mat_rot_Z = np.array([
    [np.cos(theta_1), -np.sin(theta_1), 0],
    [np.sin(theta_1),  np.cos(theta_1), 0],
    [0, 0, 1]
])

mat_rot_Y = np.array([
    [np.cos(theta_2),0,np.sin(theta_2)],
    [0,1,0],
    [-np.sin(theta_2),0,np.cos(theta_2)]
])

mat_rot_X = np.array([
    [1,0,0],
    [0,np.cos(theta_3),-np.sin(theta_3)],
    [0,np.sin(theta_3),np.cos(theta_3)]
])

rotation =  mat_rot_X @ mat_rot_Y  @ mat_rot_Z

print(rotation)

"""
Wyznaczanie punktuy posredniego:
-punkt posredni musza dwie osie przecinac sie w środku czy 4 i 5
-znać trzeba macierz rotacji względem punktu bazowgo 0
"""

Pgoal = np.array([[100],
                  [80],
                  [60]])

L4 = 10
L5 = 5

z_vec = rotation[:,2]

print(z_vec)

Pmid = Pgoal.T - (L4 + L5)*z_vec

print(Pmid)