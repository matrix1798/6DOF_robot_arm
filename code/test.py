import numpy as np
if not hasattr(np, 'disp'):
    np.disp = print
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.widgets as widgets
from kinematicTools import inverseKinematic, axial_interpolation
import time

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

start_phi = inverseKinematic(0.2, 0.2, 0.1, 0.0, 180.0, 0.0)
stop_phi = inverseKinematic(0.2, -0.2, 0.1, 0.0, 180.0, 0.0)

axial_matrix = axial_interpolation(start_phi,stop_phi,10)

# Startujemy od kątów zerowych lub z pierwszego punktu
aktualne_katy = np.zeros(6) 

env = robot.plot(aktualne_katy, block=False)

for i in range(10):
    # Oblicz IK dla punktu docelowego
    docelowe_katy = axial_matrix[i]
    
    # Wygeneruj trajektorię
    trajektoria = rtb.jtraj(aktualne_katy, docelowe_katy, 50)
    
    # 2. Animacja ruchu
    for q_step in trajektoria.q:
        # Aktualizacja pozycji robota na wykresie
        env.step(0.00001) # Mały krok czasowy dla płynności
        robot.q = q_step
        
    aktualne_katy = docelowe_katy

# 3. Zatrzymaj okno po zakończeniu ruchu, żeby nie zgasło od razu
print("Ruch zakończony.")
env.hold()