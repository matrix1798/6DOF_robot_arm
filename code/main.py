import numpy as np

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


mat_H_0_1 = DHmatrix(0,0,0,0)

print(mat_H_0_1)