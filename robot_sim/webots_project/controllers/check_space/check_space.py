import csv
import math
import time
from collections import deque
from itertools import product

import numpy as np
from controller import Supervisor

# Dopasuj ścieżkę importu do swojej struktury projektu
from kinematicTools import robot6DOF

# ================= KONFIGURACJA =================
L_0, L_1, L_2, L_3, L_4, L_5 = (120.925, 54.0, 93.255, 27.0, 103.225, 0.0)

# Siatka przestrzenna [mm]
STEP_MM = 25.0          # krok przeszukiwania [mm]
NEIGHBORS_26 = False    # False = 6 sąsiadów (±X, ±Y, ±Z), True = 26 sąsiadów
MIN_Z_MM = 0.0          # minimalna wysokość Z [mm]

# Lista orientacji efektora (roll, pitch, yaw) w stopniach do przetestowania w każdym punkcie
ROLL_LIST = [0.0, -45.0, 45.0, -90.0, 90.0]
PITCH_LIST = [0.0, -45.0, 45.0, -90.0, 90.0, 180.0]
YAW_LIST = [0.0]
ORIENTATIONS_DEG = list(product(ROLL_LIST, PITCH_LIST, YAW_LIST))

# Konfiguracje łokcia do sprawdzenia w każdym punkcie ('up', 'down')
ELBOW_CONFIGS = ["up"]

# Mapowanie kątów IK na osie silników w Webots (0° w Webots = wyprostowane ramię dh_zero)
WEBOTS_USE_DIRECTION = True

# Parametry symulacji Webots
JOINTS_NAME = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
MOTOR_VELOCITY = 10.0   # [rad/s]
SETTLE_STEPS = 12       # kroki na dojazd do pozycji docelowej
CHECK_STEPS = 3         # kroki sprawdzania kolizji w pozycji docelowej

# Plik wyjściowy
OUTPUT_CSV = "workspace_results.csv"
# ================================================


def convert_ik_to_webots(robot: robot6DOF, phi_rad):
    """
    Przelicza kąty z IK (radiany) na kąty dla silników Webots (względem dh_zero = 0),
    bez filtrowania po min_angle / max_angle.

    Zwraca: (webots_angles_rad, webots_angles_deg)
    """
    webots_rad = []
    webots_deg = []

    for i in range(6):
        angle_ik_deg = np.degrees(phi_rad[i])
        dh_offset = robot.limits_map["dh_zero"][i]
        direction = robot.limits_map["direction"][i]

        delta_phi = ((angle_ik_deg - dh_offset + 180.0) % 360.0) - 180.0
        angle_deg = (direction * delta_phi) if WEBOTS_USE_DIRECTION else delta_phi

        webots_deg.append(angle_deg)
        webots_rad.append(math.radians(angle_deg))

    return webots_rad, webots_deg


def solve_ik_safe(robot: robot6DOF, x_mm, y_mm, z_mm, r_deg, p_deg, yaw_deg, elbow):
    """Wywołuje IK z wyciszeniem warningów dla sqrt(<0) i zwraca None jeśli wynik ma NaN."""
    with np.errstate(invalid="ignore", divide="ignore"):
        phi = robot.inverseKinematic(
            x_mm, y_mm, z_mm, r_deg, p_deg, yaw_deg, elbow_pos=elbow, show_angles=False
        )
    if np.any(np.isnan(phi)) or np.any(np.isinf(phi)):
        return None
    return phi


def is_pose_collision_free(supervisor, robot_node, engines, webots_angles_rad, timestep):
    """Ustawia zadaną pozę w Webots i sprawdza, czy w pozycji docelowej występuje kolizja."""
    for engine, angle in zip(engines, webots_angles_rad):
        engine.setPosition(float(angle))

    for _ in range(SETTLE_STEPS):
        if supervisor.step(timestep) == -1:
            return None

    collision = False
    for _ in range(CHECK_STEPS):
        if supervisor.step(timestep) == -1:
            return None
        if len(robot_node.getContactPoints(True)) > 0:
            collision = True
            break

    if collision:
        robot_node.resetPhysics()

    return not collision


def get_neighbor_offsets():
    if NEIGHBORS_26:
        return [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
            if not (dx == 0 and dy == 0 and dz == 0)
        ]
    return [
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    ]


def find_start_grid_cell(robot: robot6DOF, supervisor, robot_node, engines, timestep):
    """Wyznacza punkt startowy na siatce na podstawie wyprostowanego ramienia (dh_zero)."""
    dh_zero_rad = np.radians(robot.limits_map["dh_zero"])
    x0, y0, z0, r0, p0, yaw0 = robot.forwardKinematic(dh_zero_rad)
    print(
        f"Wyprostowane ramię (FK): X={x0:.2f} mm, Y={y0:.2f} mm, Z={z0:.2f} mm | "
        f"R={np.degrees(r0):.1f}°, P={np.degrees(p0):.1f}°, Y={np.degrees(yaw0):.1f}°"
    )

    ix0 = int(round(x0 / STEP_MM))
    iy0 = int(round(y0 / STEP_MM))
    iz0 = int(math.floor(z0 / STEP_MM))

    for dz in range(0, 5):
        cell = (ix0, iy0, iz0 - dz)
        x, y, z = cell[0] * STEP_MM, cell[1] * STEP_MM, cell[2] * STEP_MM
        for r, p, yaw in ORIENTATIONS_DEG:
            for elbow in ELBOW_CONFIGS:
                phi = solve_ik_safe(robot, x, y, z, r, p, yaw, elbow)
                if phi is None:
                    continue
                webots_rad, _ = convert_ik_to_webots(robot, phi)
                ok = is_pose_collision_free(supervisor, robot_node, engines, webots_rad, timestep)
                if ok:
                    print(f"Punkt startowy siatki: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
                    return cell

    raise RuntimeError(
        "Nie znaleziono poprawnego punktu startowego przy wyprostowanym ramieniu. "
        "Sprawdź ORIENTATIONS_DEG lub STEP_MM."
    )


def main():
    robot = robot6DOF(L_0, L_1, L_2, L_3, L_4, L_5)

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    robot_node = supervisor.getSelf()
    robot_node.enableContactPointsTracking(timestep, True)

    engines = [supervisor.getDevice(name) for name in JOINTS_NAME]
    for engine in engines:
        max_vel = engine.getMaxVelocity()
        vel = min(MOTOR_VELOCITY, max_vel) if max_vel > 0 else MOTOR_VELOCITY
        engine.setVelocity(vel)

    # Pozycja początkowa: wyprostowane ramię
    for engine in engines:
        engine.setPosition(0.0)
    for _ in range(SETTLE_STEPS * 2):
        if supervisor.step(timestep) == -1:
            return

    max_reach_mm = L_1 + L_2 + L_3 + L_4 + L_5 + STEP_MM
    neighbors = get_neighbor_offsets()

    start_cell = find_start_grid_cell(robot, supervisor, robot_node, engines, timestep)

    queue = deque([start_cell])
    visited = {start_cell}

    reachable_points = 0
    reachable_poses = 0
    checked_points = 0
    t_start = time.time()

    bounds = {
        "x_min": float("inf"), "x_max": float("-inf"),
        "y_min": float("inf"), "y_max": float("-inf"),
        "z_min": float("inf"), "z_max": float("-inf"),
    }
    joint_min = [float("inf")] * 6
    joint_max = [float("-inf")] * 6

    with open(OUTPUT_CSV, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "x_mm", "y_mm", "z_mm",
            "roll_deg", "pitch_deg", "yaw_deg",
            "elbow",
            "j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg",
        ])

        sim_running = True
        while queue and sim_running:
            ix, iy, iz = queue.popleft()
            x_mm = ix * STEP_MM
            y_mm = iy * STEP_MM
            z_mm = iz * STEP_MM
            checked_points += 1

            if z_mm < MIN_Z_MM:
                continue
            dist_from_shoulder = math.sqrt(x_mm**2 + y_mm**2 + (z_mm - L_0) ** 2)
            if dist_from_shoulder > max_reach_mm:
                continue

            point_reachable = False

            for r_deg, p_deg, yaw_deg in ORIENTATIONS_DEG:
                for elbow in ELBOW_CONFIGS:
                    phi = solve_ik_safe(robot, x_mm, y_mm, z_mm, r_deg, p_deg, yaw_deg, elbow)
                    if phi is None:
                        continue

                    webots_rad, webots_deg = convert_ik_to_webots(robot, phi)

                    ok = is_pose_collision_free(
                        supervisor, robot_node, engines, webots_rad, timestep
                    )
                    if ok is None:
                        sim_running = False
                        break

                    if ok:
                        point_reachable = True
                        reachable_poses += 1
                        for i in range(6):
                            joint_min[i] = min(joint_min[i], webots_deg[i])
                            joint_max[i] = max(joint_max[i], webots_deg[i])

                        writer.writerow([
                            round(x_mm, 3), round(y_mm, 3), round(z_mm, 3),
                            round(r_deg, 2), round(p_deg, 2), round(yaw_deg, 2),
                            elbow,
                            *[round(a, 2) for a in webots_deg],
                        ])

                if not sim_running:
                    break

            if point_reachable:
                reachable_points += 1
                bounds["x_min"] = min(bounds["x_min"], x_mm)
                bounds["x_max"] = max(bounds["x_max"], x_mm)
                bounds["y_min"] = min(bounds["y_min"], y_mm)
                bounds["y_max"] = max(bounds["y_max"], y_mm)
                bounds["z_min"] = min(bounds["z_min"], z_mm)
                bounds["z_max"] = max(bounds["z_max"], z_mm)

                for dx, dy, dz in neighbors:
                    nxt = (ix + dx, iy + dy, iz + dz)
                    if nxt not in visited:
                        visited.add(nxt)
                        queue.append(nxt)

            if checked_points % 25 == 0:
                f.flush()
                elapsed = time.time() - t_start
                print(
                    f"Sprawdzono: {checked_points} | Osiągalne pkt: {reachable_points} | "
                    f"Pozy (pkt+kąt): {reachable_poses} | Kolejka: {len(queue)} | Czas: {elapsed:.1f}s"
                )

    elapsed = time.time() - t_start
    print("\n===== PODSUMOWANIE PRZESTRZENI ROBOCZEJ =====")
    print(f"Zapisano do: {OUTPUT_CSV}")
    print(f"Sprawdzone punkty siatki: {checked_points}")
    print(f"Osiągalne punkty (X,Y,Z): {reachable_points}")
    print(f"Osiągalne pozy (X,Y,Z,R,P,Y): {reachable_poses}")
    if reachable_points > 0:
        print(f"Zakres X [mm]: {bounds['x_min']:.1f} .. {bounds['x_max']:.1f}")
        print(f"Zakres Y [mm]: {bounds['y_min']:.1f} .. {bounds['y_max']:.1f}")
        print(f"Zakres Z [mm]: {bounds['z_min']:.1f} .. {bounds['z_max']:.1f}")
        print("\nBezkolizyjne zakresy kątów złączy (Webots [deg]):")
        for i in range(6):
            print(f"  {JOINTS_NAME[i]}: {joint_min[i]:7.2f}° .. {joint_max[i]:7.2f}°")
    print(f"Czas całkowity: {elapsed:.1f}s")


if __name__ == "__main__":
    main()