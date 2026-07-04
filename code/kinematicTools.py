import mathTools as mt
import numpy as np


class robot6DOF:

        """
        A class used to simulate a robot arm with a specific design derived using the DH method.
        """

        def __init__(self,L_0, L_1, L_2, L_3, L_4, L_5):
                self.L_0 = L_0
                self.L_1 = L_1
                self.L_2 = L_2
                self.L_3 = L_3
                self.L_4 = L_4
                self.L_5 = L_5

        def inverseKinematic(self,X,Y,Z,roll,pitch,yaw, elbow_pos:str = 'up', show_angels = True):

                """
                Function for calculating joint angle values ​​based on
                the given end-effector position and orientation relative to the base frame
                
                Input:
                X, Y, Z - system coordinates in meteres
                Angels of joints in radians
                """

                X = X
                Y = Y
                Z = Z
                r = roll
                p = pitch
                yaw = yaw

                P_goal = np.array([X,Y,Z,np.radians(r),np.radians(p),np.radians(yaw)])

                Rotation_goal = mt.rotation_mat(roll = r, pitch = p, yaw = yaw)
                P_wc = P_goal[0:3] - Rotation_goal@np.array([0,0,self.L_4+self.L_5]).T

                #phi_0
                phi_0 = np.atan2(P_wc[1],P_wc[0])

                #auxiliary vwariables
                x = P_wc[0]
                y = P_wc[1]

                R = np.sqrt(x**2 + y**2)
                z = P_wc[2] - self.L_0
                L_23 = self.L_2 + self.L_3
                D = (self.L_1**2 + L_23**2 - R**2 - z**2)/(2*self.L_1*L_23)

                #phi_2 +/- depends on elbow position (- for elbow up)
                if elbow_pos == 'up':
                        phi_2 = np.atan2(D,-np.sqrt(1-D**2))
                else:
                        phi_2 = np.atan2(D,np.sqrt(1-D**2))
                #phi_1
                k1 = self.L_1 - L_23*np.sin(phi_2)
                k2 = L_23*np.cos(phi_2)
                phi_1 = np.atan2(k1*z - k2*R, k1*R + k2*z)

                #rotation of joint 0 to 3 
                mat_rot_0_1 = np.array([
                        [np.cos(phi_0), 0,  np.sin(phi_0),0],
                        [np.sin(phi_0), 0, -np.cos(phi_0),0],
                        [  0 , 1,   0 ,self.L_0],
                        [0,0,0,1]
                        ])

                mat_rot_1_2 = np.array([
                        [np.cos(phi_1), -np.sin(phi_1),  0,self.L_1*np.cos(phi_1)],
                        [np.sin(phi_1), np.cos(phi_1), 0, self.L_1*np.sin(phi_1)],
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

                # If the wrist is extended (a distinctive feature)
                if sin_phi4 < 1e-6:
                        phi_4 = 0.0
                        phi_3 = 0.0
                        # phi_5 takes over the full rotation of the wrist
                        phi_5 = np.atan2(R_3_6[1, 0], R_3_6[0, 0])
                else:
                        # Standard compute
                        phi_4 = np.atan2(sin_phi4, cos_phi4)
                        phi_3 = np.atan2(-R_3_6[1, 2], -R_3_6[0, 2])
                        phi_5 = np.atan2(-R_3_6[2, 1], R_3_6[2, 0])
                
                if show_angels:
                        print(f'\nGoal position: x: {X}m, y: {Y}, z: {Z}\nroll: {r}, pitch: {p}, yaw: {yaw}')
                        print(f'Phi_0 = {np.degrees(phi_0):.3f}')
                        print(f'Phi_1 = {np.degrees(phi_1):.3f}')
                        print(f'Phi_2 = {np.degrees(phi_2):.3f}')
                        print(f'Phi_3 = {np.degrees(phi_3):.3f}')
                        print(f'Phi_4 = {np.degrees(phi_4):.3f}')
                        print(f'Phi_5 = {np.degrees(phi_5):.3f}')

                return phi_0, phi_1, phi_2, phi_3, phi_4, phi_5

        @classmethod
        def axial_interpolation(cls,start_phi,stop_phi,accuracy):

                phi_list = []

                for i in range(len(start_phi)):
                        phi_vec = np.linspace(start_phi[i],stop_phi[i],accuracy)
                        phi_list.append(phi_vec)

                phi_matrix = np.column_stack(phi_list)
                
                return phi_matrix
