from tools import rotation_mat
import numpy as np

def inverseKinematic(X,Y,Z,roll,pitch,yaw):

        """
        Funkcja do obliczenia wartości kątowych złączy na podstawie
        podanej pozycji efektora i oreintacji względem układu bazowego

        Wyjście:
        kąty złączy w radianach
        """

        X = X
        Y = Y
        Z = Z
        r = roll
        p = pitch
        yaw = yaw

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

        print(f'\nPozycja celu: x: {X}m, y: {Y}, z: {Z}\n roll: {r}, pitch: {p}, yaw: {yaw}')
        print(f'Phi_0 = {np.degrees(phi_0)}')
        print(f'Phi_1 = {np.degrees(phi_1)}')
        print(f'Phi_2 = {np.degrees(phi_2)}')
        print(f'Phi_3 = {np.degrees(phi_3)}')
        print(f'Phi_4 = {np.degrees(phi_4)}')
        print(f'Phi_5 = {np.degrees(phi_5)}')

        return phi_0, phi_1, phi_2, phi_3, phi_4, phi_5