import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
from kinematicTools import robot6DOF

"""
This test checks for correctness implemented equals of inverse kinematic
with visualization on simple model of robot arm
"""

# Define robot with RT
L_0, L_1, L_2, L_3, L_4, L_5 = 0.15, 0.20, 0.15, 0.15, 0.15, 0.05

robot = rtb.DHRobot([
    rtb.RevoluteDH(d=L_0,      a=0,   alpha=np.pi/2), 
    rtb.RevoluteDH(d=0,        a=L_1, alpha=0),      
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi/2),       
    rtb.RevoluteDH(d=L_2+L_3,  a=0,   alpha=np.pi/2),
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi/2),
    rtb.RevoluteDH(d=L_4+L_5,  a=0,   alpha=0)
], name="Robot 6DOF")

# Initi object of my implementation
robot_IK = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

# Init 3D env
env = PyPlot()
env.launch()
env.add(robot)

env.fig.subplots_adjust(left=0.3)

text_pos = env.ax.text2D(0.05, 0.85, "", transform=env.ax.transAxes, 
                              fontsize=11, bbox=dict(facecolor='white', alpha=0.8))
# gui
ax_x = env.fig.add_axes([0.05, 0.75, 0.15, 0.05])
ax_y = env.fig.add_axes([0.05, 0.65, 0.15, 0.05])
ax_z = env.fig.add_axes([0.05, 0.55, 0.15, 0.05])
ax_r = env.fig.add_axes([0.05, 0.45, 0.15, 0.05])
ax_p = env.fig.add_axes([0.05, 0.35, 0.15, 0.05])
ax_yw = env.fig.add_axes([0.05, 0.25, 0.15, 0.05])
ax_btn = env.fig.add_axes([0.05, 0.10, 0.15, 0.08])

txt_x = TextBox(ax_x, 'X: ', initial='0.4')
txt_y = TextBox(ax_y, 'Y: ', initial='0.0')
txt_z = TextBox(ax_z, 'Z: ', initial='0.1')
txt_r = TextBox(ax_r, 'Roll: ', initial='0.0')
txt_p = TextBox(ax_p, 'Pitch: ', initial='180.0')
txt_yw = TextBox(ax_yw, 'Yaw: ', initial='0.0')
btn_move = Button(ax_btn, 'Execute move')

trajectory_queue = []
current_q = np.zeros(6)

def on_click(event):
    global trajectory_queue, current_q
    try:
        x, y, z = float(txt_x.text), float(txt_y.text), float(txt_z.text)
        r, p, yw = float(txt_r.text), float(txt_p.text), float(txt_yw.text)

        print(f"Computing IK for: X={x}, Y={y}, Z={z}, R={r}, P={p}, Yw={yw}")

        phi_stop = robot_IK.inverseKinematic(x, y, z, r, p, yw)
        phi_matrix = robot6DOF.axialInterpolation(current_q, phi_stop, 100)

        trajectory_queue = phi_matrix.tolist()

    except ValueError:
        print("Error: Make sure you enter only numbers (use dots, not commas)!")

btn_move.on_clicked(on_click)

# main loop

while plt.fignum_exists(env.fig.number):

    if len(trajectory_queue) > 0:
        current_q = trajectory_queue.pop(0)
        robot.q = current_q

        T = robot.fkine(current_q)
        ef_x, ef_y, ef_z = T.t
        text_pos.set_text(f"Efektor (Live):\nX: {ef_x:.3f} m\nY: {ef_y:.3f} m\nZ: {ef_z:.3f} m")

    env.step(0.02)