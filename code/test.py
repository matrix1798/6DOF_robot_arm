import numpy as np
if not hasattr(np, 'disp'):
    np.disp = print
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.widgets as widgets
from tools import rotation_mat
from IKequals import phi_0, phi_1, phi_2, phi_3, phi_4, phi_5 

# Łatka naprawiająca suwaki w nowym matplotlib
oryginalny_slider = widgets.Slider.__init__
def naprawiony_slider(self, ax, label, valmin, valmax, valinit=0.5, valfmt="%1.2f", **kwargs):
    oryginalny_slider(self, ax=ax, label=label, valmin=valmin, valmax=valmax, valinit=valinit, valfmt=valfmt, **kwargs)

widgets.Slider.__init__ = naprawiony_slider

# --- KONFIGURACJA ROBOTA ---
L_0, L_1, L_2, L_3, L_4, L_5 = 0.15, 0.20, 0.15, 0.15, 0.15, 0.05
L_23 = L_2 + L_3

robot = rtb.DHRobot([
    rtb.RevoluteDH(d=L_0,     a=0,   alpha=np.pi/2), 
    rtb.RevoluteDH(d=0,       a=L_1, alpha=0),      
    rtb.RevoluteDH(d=0,       a=0,   alpha=-np.pi/2),       
    rtb.RevoluteDH(d=L_2+L_3, a=0,   alpha=np.pi/2),
    rtb.RevoluteDH(d=0,       a=0,   alpha=-np.pi/2),
    rtb.RevoluteDH(d=L_4+L_5, a=0,   alpha=0)
], name="Moj Robot")

"""
phi_0 = np.radians(71.56)
phi_1 = np.radians(56.81)
phi_2 = np.radians(166.75)
phi_3 = np.radians(155.29)
phi_4 = np.radians(130.82)
phi_5 = np.radians(73.26)
"""
angles = [phi_0, phi_1, phi_2, phi_3, phi_4, phi_5]

robot.teach(angles)