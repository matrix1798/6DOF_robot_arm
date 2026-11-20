import numpy as np

np.set_printoptions(precision=4, suppress=True)

def DHmatrix (phi_i,delta_i,lambda_i,theta_i):

    """
    Macierz Denavita-Hartenberga(DH):

    i-1_H_i = np.array([
                        [cos(phi_i), -sin(phi_i)*cos(theta_i), sin(phi_i)*sin(theta_i) , gammma_i*cos(phi_i)]
                        [sin(phi_i),  cos(phi_i)*cos(theta_i), -cos(phi_i)*sin(theta_i), gammma_i*sin(phi_i)]
                        [     0    ,        sin(theta_i)     ,        cos(theta_i)     ,       delta_i      ]
                        [     0    ,             0           ,             0           ,          1         ]
                        ])
    """

    return np.array([
              [ np.cos(phi_i), -np.sin(phi_i)*np.cos(theta_i) , np.sin(phi_i)*np.sin(theta_i) , lambda_i*np.cos(phi_i)],
              [ np.sin(phi_i), np.cos(phi_i)*np.cos(theta_i)  , -np.cos(phi_i)*np.sin(theta_i), lambda_i*np.sin(phi_i)],
              [     0        ,          np.sin(theta_i)       ,           np.cos(theta_i)     ,      delta_i          ],
              [     0        ,              0                 ,                 0             ,         1             ]
              ])

def rotation_mat (roll,pitch,yaw):

    """
    roll - obrót wokół osi X
    pitch - obrót wokół osi Y
    yaw - obrót wokół osi Z

    w stopniach

    Kolejność mnożenia mat_rot_Z @ mat_rot_Y @ mat_rot_X 
    jest prawidłowa dla standardowej konwencji Roll-Pitch-Yaw (tzw. sekwencja Tait-Bryan Z-Y-X).
    """

    theta_1 = np.radians(yaw)
    theta_2 = np.radians(pitch)
    theta_3 = np.radians(roll)

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

    rotation =  mat_rot_Z @ mat_rot_Y  @  mat_rot_X 

    return rotation

def rollPitchYawToQuaternion(roll, pitch, yaw):
    """
    Input:
    roll, pitch, yaw - degrees

    Output (quaternion in form):
    q = w + x*i + y*j + z*k
    """

    r = np.radians(roll)
    p = np.radians(pitch)
    yw = np.radians(yaw)

    sin_r = np.sin(r/2.0)
    cos_r = np.cos(r/2.0)
    sin_p = np.sin(p/2.0)
    cos_p = np.cos(p/2.0)
    sin_yw = np.sin(yw/2.0)
    cos_yw = np.cos(yw/2.0)

    w = cos_r*cos_p*cos_yw + sin_r*sin_p*sin_yw
    x = sin_r*cos_p*cos_yw - cos_r*sin_p*sin_yw
    y = cos_r*sin_p*cos_yw + sin_r*cos_p*sin_yw
    z = cos_r*cos_p*sin_yw - sin_r*sin_p*cos_yw

    return w, x, y, z

def quaternionToRollPitchYaw (w, x, y ,z):

    tolernace = 0.999

    p = 2*(w*y - z*x)

    # anti gimbal lock
    if abs(p) >= tolernace:
        roll = 0.0
        if p < 0:
            pitch = -90.0
            yaw = -2.0*np.atan2(x,w) * 180 / np.pi
        else:
            pitch = 90.0
            yaw = 2.0*np.atan2(x,w) * 180 / np.pi
        
    else:
        roll = np.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2)) * 180 / np.pi
        pitch = np.arcsin(p) * 180 / np.pi
        yaw = np.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2)) * 180 / np.pi

    return roll, pitch, yaw