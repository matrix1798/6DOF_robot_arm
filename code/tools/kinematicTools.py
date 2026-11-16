import code.tools.mathTools as mt
import numpy as np
from .mathTools import rollPitchYawToQuaternion, quaternionToRollPitchYaw

class robot6DOF:

        """
        A class used to simulate a robot arm with a specific design derived using the DH method.
        """

        def __init__(self,L_0, L_1, L_2, L_3, L_4, L_5, limits_map = None):
                self.L_0 = L_0
                self.L_1 = L_1
                self.L_2 = L_2
                self.L_3 = L_3
                self.L_4 = L_4
                self.L_5 = L_5
                
                if limits_map == None:
                        self.limits_map = {
                                        'min_angle': [ 40.0,  90.0,  60.0,   0.0,  25.0,   0.0],
                                        'max_angle': [320.0, 290.0, 300.0, 360.0, 280.0, 360.0],
                                        'offset':    [ -4.0,   3.0,   1.0, -42.0, -26.0,   0.0],
                                        'dh_zero':   [  0.0,  90.0, -90.0, 180.0,  0.0,    0.0],
                                        'direction': [   -1,     1,     1,     -1,   -1,     1]
                                        }
                else:
                        self.limits_map = limits_map

        def forwardKinematic(self,angles):
                """
                Function fro calculating position and orientation of effecter based
                on angles in joints with use DH method
                """

                X = 0.0
                Y = 0.0
                Z = 0.0
                roll = 0.0
                pitch = 0.0
                yaw = 0.0

                DHtable = [[angles[0],           self.L_0,         0,  np.pi/2],
                           [angles[1],                   0, self.L_1,        0],
                           [angles[2],                   0,        0, -np.pi/2],
                           [angles[3], self.L_2 + self.L_3,        0,  np.pi/2],
                           [angles[4],                   0,        0, -np.pi/2],
                           [angles[5], self.L_4 + self.L_5,        0,        0]]

                H_mat_result = mt.DHmatrix(*DHtable[0])

                for i in range(1,6):
                        H_mat_result = H_mat_result @ mt.DHmatrix(*DHtable[i])

                X = H_mat_result[0,3]
                Y = H_mat_result[1,3]
                Z = H_mat_result[2,3]                
                
                roll = np.atan2(H_mat_result[2,1], H_mat_result[2,2])
                pitch = np.arcsin(-1 *  H_mat_result[2,0])
                yaw = np.atan2(H_mat_result[1,0], H_mat_result[0,0])

                return X, Y, Z, roll, pitch, yaw



        def inverseKinematic(self,X,Y,Z,roll,pitch,yaw, elbow_pos:str = 'up', show_angles = True):

                """
                Function for calculating joint angle values ​​based on
                the given end-effector position and orientation relative to the base frame
                
                Input:
                X, Y, Z - system coordinates in meteres
                angles of joints in degrees
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
                
                if show_angles:
                        print(f'Goal position: x: {X}, y: {Y}, z: {Z}\nroll: {r}, pitch: {p}, yaw: {yaw}')
                        print(f'Phi_0 = {np.degrees(phi_0):.3f}')
                        print(f'Phi_1 = {np.degrees(phi_1):.3f}')
                        print(f'Phi_2 = {np.degrees(phi_2):.3f}')
                        print(f'Phi_3 = {np.degrees(phi_3):.3f}')
                        print(f'Phi_4 = {np.degrees(phi_4):.3f}')
                        print(f'Phi_5 = {np.degrees(phi_5):.3f}')

                return [phi_0, phi_1, phi_2, phi_3, phi_4, phi_5]

        def axialInterpolation(self, P_start:list, P_goal:list, accuracy, max_v, max_a = None):

                """
                Input:
                P_start  - value of start position of effector
                P_goal   - value of goal position of effector
                accuracy - number of steps between start and goal position of each joint
                max_v    - max velocity of each servo [rad/s]

                """

                start_phi = self.inverseKinematic(*P_start, show_angles=False)
                goal_phi = self.inverseKinematic(*P_goal, show_angles=False)

                dif_phi = np.array(start_phi) - np.array(goal_phi)
                abs_dif_phi = np.abs(dif_phi)

                idx = np.argmax(abs_dif_phi)

                if abs_dif_phi[idx] < 1e-3:
                        return [0.0] * 6, goal_phi

                time_total = abs_dif_phi[idx] / max_v

                new_speed_list = []
                for i in range(6):
                        new_speed = abs_dif_phi[i]/time_total
                        new_speed_list.append(new_speed)

                return new_speed_list, goal_phi

        def linearInterpolation(self, P_start:list, P_goal:list, accuracy):
                """
                Input:
                P_0/1 = [x, y, z, roll, pitch, yaw]
                x, y, z - in meters
                roll, pitch, yaw - degrees
                """

                P_0 = np.array(P_start[:3])
                w_0, x_0, y_0, z_0 = rollPitchYawToQuaternion(P_start[3], P_start[4], P_start[5])
                
                P_1 = np.array(P_goal[:3])
                w_1, x_1, y_1, z_1 = rollPitchYawToQuaternion(P_goal[3], P_goal[4], P_goal[5])
                
                steps = np.linspace(0.0,1.0,accuracy)
                q_list = []
                P_s = np.zeros(3)
                dif_P = P_1-P_0
                tolerance = 0.0005
                w_s = 0.0
                x_s = 0.0
                y_s = 0.0
                z_s = 0.0

                for s in steps:
                        P_s = P_0 + s * dif_P

                        cos_omega = w_0*w_1 + x_0*x_1 + y_0*y_1 + z_0*z_1

                        if cos_omega < 0:
                                w_1, x_1, y_1, z_1 = -w_1, -x_1, -y_1, -z_1
                                cos_omega = -cos_omega
                        
                        if (1-cos_omega) < tolerance:
                                w_s = ((1-s)*w_0 + s*w_1)
                                x_s = ((1-s)*x_0 + s*x_1)
                                y_s = ((1-s)*y_0 + s*y_1)
                                z_s = ((1-s)*z_0 + s*z_1)

                                norm = np.sqrt(w_s**2 + x_s**2 + y_s**2 + z_s**2)

                                w_s /= norm
                                x_s /= norm
                                y_s /= norm
                                z_s /= norm
                        else:
                                omega = np.arccos(cos_omega)
                                sin_omega = np.sin(omega)
                                w_s = np.sin((1-s)*omega)/sin_omega * w_0 + np.sin(s*omega)/sin_omega * w_1
                                x_s = np.sin((1-s)*omega)/sin_omega * x_0 + np.sin(s*omega)/sin_omega * x_1
                                y_s = np.sin((1-s)*omega)/sin_omega * y_0 + np.sin(s*omega)/sin_omega * y_1
                                z_s = np.sin((1-s)*omega)/sin_omega * z_0 + np.sin(s*omega)/sin_omega * z_1

                        roll, pitch, yaw = quaternionToRollPitchYaw(w_s, x_s, y_s, z_s)

                        q_s = self.inverseKinematic(P_s[0], P_s[1], P_s[2], roll, pitch, yaw,show_angles=True)
                        q_list.append(q_s)

                q_matrix = np.column_stack(q_list)

                return q_matrix

        def angleConvert (self,angles):

                angles_coverted = []

                for i in range(6):
                        angle_IK = np.degrees(angles[i]) 
                        max_angle = self.limits_map['max_angle'][i]
                        min_angle = self.limits_map['min_angle'][i]
                        mech_offset = self.limits_map['offset'][i]
                        dh_offset = self.limits_map['dh_zero'][i]
                        direction = self.limits_map['direction'][i]

                        delta_phi = ((angle_IK - dh_offset + 180.0) % 360.0) - 180.0
                        angle_convert = 180.8 + direction*delta_phi + mech_offset

                        if angle_convert > max_angle:
                                angle_convert = max_angle
                        elif angle_convert < min_angle:
                                angle_convert = min_angle

                        angles_coverted.append(angle_convert)

                return angles_coverted      