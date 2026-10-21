import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

from spatialmath import SE3
from matplotlib.widgets import TextBox, Button

from ..tools.kinematicTools import robot6DOF


# =========================================================
# 1. KONFIGURACJA ROBOTA DH
# =========================================================

L_0, L_1, L_2, L_3, L_4, L_5 = (
    120.925, 54, 93.255, 27, 103.225, 0.0
)

robot = rtb.DHRobot([
    rtb.RevoluteDH(d=L_0,      a=0,   alpha=np.pi / 2),
    rtb.RevoluteDH(d=0,        a=L_1, alpha=0),
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi / 2),
    rtb.RevoluteDH(d=L_2 + L_3, a=0,  alpha=np.pi / 2),
    rtb.RevoluteDH(d=0,        a=0,   alpha=-np.pi / 2),
    rtb.RevoluteDH(d=L_4 + L_5, a=0,  alpha=0)
], name="Robot 6DOF")

robot_6DOF = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)


# =========================================================
# 2. POCZĄTKOWA TRAJEKTORIA
#    XYZ w mm, RPY w stopniach
# =========================================================

P_1 = [100, 100, 100, 180, 0, 0]
P_2 = [100,   0, 100, 180, 0, 0]
P_3 = [  0,-100,  80, 180, 0, 0]

P1_to_P2 = robot_6DOF.linearInterpolation(P_1, P_2, 100)
P2_to_P3 = robot_6DOF.linearInterpolation(P_2, P_3, 100)

phi_matrix = np.hstack((P1_to_P2, P2_to_P3))


# =========================================================
# 3. WIZUALIZACJA
# =========================================================

plt.ion()

fig = plt.figure(figsize=(12, 8))
ax = fig.add_axes([0.05, 0.20, 0.58, 0.75],
                  projection="3d")

ax.set_xlim([-200, 300])
ax.set_ylim([-250, 250])
ax.set_zlim([0, 400])

ax.set_xlabel("X [mm]")
ax.set_ylabel("Y [mm]")
ax.set_zlabel("Z [mm]")
ax.set_title("Robot 6DOF")

# Zachowanie proporcji osi, jeśli obsługuje je wersja Matplotlib
try:
    ax.set_box_aspect((1, 1, 1))
except AttributeError:
    pass

# Szkielet robota bez układów współrzędnych
robot_line, = ax.plot(
    [], [], [],
    "-o", color="royalblue",
    linewidth=4, markersize=6
)

# Punkt docelowy efektora
target_point, = ax.plot(
    [], [], [], "ro", markersize=8
)

# Informacje o robocie
info_box = fig.text(
    0.67, 0.90, "",
    fontsize=10,
    family="monospace",
    verticalalignment="top",
    bbox=dict(facecolor="white", alpha=0.9)
)

status_box = fig.text(
    0.67, 0.60, "Gotowy",
    fontsize=10,
    wrap=True
)


# =========================================================
# 4. POLA DO WPISYWANIA POZYCJI XYZ RPY
# =========================================================

field_names = ["X [mm]", "Y [mm]", "Z [mm]",
               "Roll [deg]", "Pitch [deg]", "Yaw [deg]"]

initial_values = ["100", "100", "100", "180", "0", "0"]

text_boxes = []

for i, (name, value) in enumerate(zip(field_names, initial_values)):
    y = 0.145 - i * 0.027

    field_ax = fig.add_axes([0.13, y, 0.13, 0.023])

    textbox = TextBox(
        field_ax,
        name,
        initial=value
    )

    text_boxes.append(textbox)


# Przycisk uruchomienia ruchu
button_ax = fig.add_axes([0.40, 0.035, 0.17, 0.07])
move_button = Button(button_ax, "JEDŹ DO XYZRPY")


# =========================================================
# 5. STAN ROBOTA
# =========================================================

current_q = np.zeros(6)
target_queue = []


# =========================================================
# 6. AKTUALIZACJA WIZUALIZACJI I ODCZYT KĄTÓW
# =========================================================

def update_robot(q):
    global current_q

    current_q = np.array(q, dtype=float).copy()

    # Transformacje kolejnych członów robota
    T_all = robot.fkine_all(current_q)

    # Pozycje przegubów i końców członów
    pts = np.asarray(T_all.t)

    robot_line.set_data(pts[:, 0], pts[:, 1])
    robot_line.set_3d_properties(pts[:, 2])

    # Pozycja i orientacja efektora
    T_end = robot.fkine(current_q)

    x, y, z = T_end.t
    rpy = T_end.rpy(unit="deg", order="zyx")

    joint_text = "\n".join(
        f"J{i + 1}: {np.rad2deg(angle):8.2f} deg"
        for i, angle in enumerate(current_q)
    )

    info_box.set_text(
        "KĄTY PRZEGUBÓW\n"
        "--------------------\n"
        f"{joint_text}\n\n"
        "EFEKTOR XYZ [mm]\n"
        "--------------------\n"
        f"X: {x:8.2f}\n"
        f"Y: {y:8.2f}\n"
        f"Z: {z:8.2f}\n\n"
        "EFEKTOR RPY [deg]\n"
        "--------------------\n"
        f"Roll:  {rpy[0]:8.2f}\n"
        f"Pitch: {rpy[1]:8.2f}\n"
        f"Yaw:   {rpy[2]:8.2f}"
    )

    fig.canvas.draw_idle()


# =========================================================
# 7. OBSŁUGA WPISYWANIA XYZ RPY
# =========================================================

def go_to_position(event):
    global target_queue

    try:
        values = np.array([
            float(box.text) for box in text_boxes
        ])

        x, y, z, roll, pitch, yaw = values

        # Zadana transformacja efektora
        T_goal = (
            SE3.Trans(x, y, z)
            * SE3.RPY(
                [roll, pitch, yaw],
                order="zyx",
                unit="deg"
            )
        )

        # Odwrotna kinematyka z aktualnej konfiguracji
        solution = robot.ikine_LM(
            T_goal,
            q0=current_q
        )

        if not solution.success:
            status_box.set_text(
                "Brak rozwiązania IK dla zadanej pozycji."
            )
            fig.canvas.draw_idle()
            return

        q_target = np.asarray(solution.q, dtype=float)

        # Dodanie celu do kolejki ruchu
        target_queue.append(q_target)

        status_box.set_text(
            f"Zadano: X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm"
        )

        target_point.set_data([x], [y])
        target_point.set_3d_properties([z])

        fig.canvas.draw_idle()

    except (ValueError, TypeError) as exc:
        status_box.set_text(f"Błędne dane wejściowe: {exc}")
        fig.canvas.draw_idle()

    except Exception as exc:
        status_box.set_text(f"Błąd kinematyki: {exc}")
        fig.canvas.draw_idle()


move_button.on_clicked(go_to_position)


# =========================================================
# 8. ANIMACJA RUCHU DO KOLEJNEJ KONFIGURACJI
# =========================================================

def animate_to(q_target, steps=80):
    global current_q

    q_start = current_q.copy()

    for i in range(1, steps + 1):
        if not plt.fignum_exists(fig.number):
            return

        alpha = i / steps

        # Interpolacja liniowa kątów przegubów
        q = q_start + alpha * (q_target - q_start)

        update_robot(q)

        fig.canvas.draw_idle()
        plt.pause(0.02)


# =========================================================
# 9. START I OBSŁUGA OKNA
# =========================================================

update_robot(current_q)

plt.show(block=False)

# Najpierw wykonaj początkową trajektorię.
for q in phi_matrix.T:
    if not plt.fignum_exists(fig.number):
        break

    update_robot(q)
    plt.pause(0.02)

# Następnie czekaj na kolejne zadane pozycje.
while plt.fignum_exists(fig.number):
    if target_queue:
        q_target = target_queue.pop(0)
        animate_to(q_target)

    plt.pause(0.02)

plt.ioff()