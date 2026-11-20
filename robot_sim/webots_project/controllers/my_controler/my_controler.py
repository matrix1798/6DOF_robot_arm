from controller import Supervisor
import numpy as np

# ===== KONFIGURACJA =====
TARGET_ANGLES_DEG = [0, 35, -180, 0, 0, 0]   # pozycja docelowa (w stopniach)
MOTOR_VELOCITY = 0.25                        # prędkość serw [rad/s], mniejsza = wolniejszy ruch
HOME_TIME = 2.0                             # czas na dojazd do pozycji zerowej [s]
MOVE_TIME = 6.0                             # czas na dojazd do pozycji docelowej [s]
# ========================

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

robot_node = supervisor.getSelf()
robot_node.enableContactPointsTracking(timestep, True)

joints_name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
engines = [supervisor.getDevice(name) for name in joints_name]

for engine in engines:
    engine.setVelocity(MOTOR_VELOCITY)

collision_counter = 0


def set_angles(angles_deg):
    for engine, angle in zip(engines, np.radians(angles_deg)):
        engine.setPosition(float(angle))


def run_and_check(seconds):
    """Wykonuje kroki symulacji i w każdym kroku sprawdza kolizję."""
    global collision_counter
    steps = int(seconds * 1000 / timestep)
    for _ in range(max(1, steps)):
        if supervisor.step(timestep) == -1:
            return False
        if len(robot_node.getContactPoints(True)) > 0:
            collision_counter += 1
            print(f"Wykryto kolizję! Iteracja: {collision_counter}")
    return True


# 1. Pozycja początkowa: wszystkie serwa na 0
print("Ustawianie pozycji początkowej (wszystkie serwa = 0)...")
set_angles([0, 0, 0, 0, 0, 0])
run_and_check(HOME_TIME)

# 2. Ruch do pozycji docelowej
print(f"Ruch do pozycji docelowej: {TARGET_ANGLES_DEG}")
set_angles(TARGET_ANGLES_DEG)
run_and_check(MOVE_TIME)

print(f"Koniec ruchu. Łączna liczba iteracji z kolizją: {collision_counter}")

# 3. Dalej monitorujemy kolizje w pozycji końcowej
while run_and_check(1.0):
    pass