import matplotlib
matplotlib.use("TkAgg")

import serial
from serial.tools import list_ports
import time
import re
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# =========================
# SETTINGS
# =========================

BAUD_RATE = 115200
MANUAL_PORT = None        # Example: "COM3". Leave as None for auto port.
UPDATE_MS = 20

GYRO_WEIGHT = 0.96        # 0.90 to 0.98 usually works
MOVEMENT_SCALE = 1.0      # increase if motion looks too small

PRINT_DEBUG_LINES = True  # set False after you confirm it works
CALIBRATION_SECONDS = 3.0
CALIBRATION_SAMPLES_REQUIRED = 50

gyro_bias_x = 0.0
gyro_bias_y = 0.0
gyro_bias_z = 0.0

accel_zero_roll = 0.0
accel_zero_pitch = 0.0

# =========================
# SERIAL PORT
# =========================

def choose_port():
    ports = list(list_ports.comports())

    if not ports:
        raise RuntimeError("No serial ports found.")

    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"{i}: {p.device} - {p.description}")

    return ports[0].device


SERIAL_PORT = MANUAL_PORT if MANUAL_PORT else choose_port()

print("Using:", SERIAL_PORT)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
time.sleep(2)
ser.reset_input_buffer()


# =========================
# PARSE YOUR ARDUINO OUTPUT
# =========================

def parse_sparkfun_line(line):
    """
    Parses lines from SparkFun ICM-20948 Example1_Basics.

    Expected line contains:
    Scaled. Acc (mg) [ ax, ay, az ], Gyr (DPS) [ gx, gy, gz ], Mag (uT) [ mx, my, mz ], Tmp (C) [ temp ]
    """

    if "Scaled. Acc" not in line:
        return None

    nums = re.findall(r"[-+]?\d+\.\d+|[-+]?\d+", line)

    if len(nums) < 10:
        return None

    ax_mg = float(nums[0])
    ay_mg = float(nums[1])
    az_mg = float(nums[2])

    gx_dps = float(nums[3])
    gy_dps = float(nums[4])
    gz_dps = float(nums[5])

    mx = float(nums[6])
    my = float(nums[7])
    mz = float(nums[8])

    temp_c = float(nums[9])

    # Convert accel from mg to g
    ax_g = ax_mg / 1000.0
    ay_g = ay_mg / 1000.0
    az_g = az_mg / 1000.0

    # Convert gyro from degrees/sec to radians/sec
    gx = math.radians(gx_dps)
    gy = math.radians(gy_dps)
    gz = math.radians(gz_dps)

    return ax_g, ay_g, az_g, gx, gy, gz, mx, my, mz, temp_c


def read_latest_packet():
    latest = None

    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        if PRINT_DEBUG_LINES and line:
            print("SERIAL:", line)

        packet = parse_sparkfun_line(line)

        if packet is not None:
            latest = packet

    return latest

def calibrate_imu():
    global gyro_bias_x, gyro_bias_y, gyro_bias_z
    global accel_zero_roll, accel_zero_pitch
    global roll, pitch, yaw

    print()
    print("======================================")
    print("CALIBRATION")
    print("Keep the IMU flat and completely still")
    print("======================================")
    print()

    time.sleep(1)

    samples = []

    start_time = time.time()

    while time.time() - start_time < CALIBRATION_SECONDS:
        packet = read_latest_packet()

        if packet is None:
            continue

        ax_g, ay_g, az_g, gx, gy, gz, mx, my, mz, temp_c = packet
        samples.append(packet)

        print(f"Calibrating... samples: {len(samples)}", end="\r")

        time.sleep(0.005)

    print()

    if len(samples) < CALIBRATION_SAMPLES_REQUIRED:
        print("WARNING: Not enough calibration samples.")
        print("Calibration skipped.")
        return

    ax_values = [s[0] for s in samples]
    ay_values = [s[1] for s in samples]
    az_values = [s[2] for s in samples]

    gx_values = [s[3] for s in samples]
    gy_values = [s[4] for s in samples]
    gz_values = [s[5] for s in samples]

    avg_ax = sum(ax_values) / len(ax_values)
    avg_ay = sum(ay_values) / len(ay_values)
    avg_az = sum(az_values) / len(az_values)

    gyro_bias_x = sum(gx_values) / len(gx_values)
    gyro_bias_y = sum(gy_values) / len(gy_values)
    gyro_bias_z = sum(gz_values) / len(gz_values)

    accel_zero_roll = math.atan2(avg_ay, avg_az)
    accel_zero_pitch = math.atan2(-avg_ax, math.sqrt(avg_ay * avg_ay + avg_az * avg_az))

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    print("Calibration complete.")
    print(f"Gyro bias X: {gyro_bias_x:.6f}")
    print(f"Gyro bias Y: {gyro_bias_y:.6f}")
    print(f"Gyro bias Z: {gyro_bias_z:.6f}")
    print(f"Zero roll:   {math.degrees(accel_zero_roll):.2f} deg")
    print(f"Zero pitch:  {math.degrees(accel_zero_pitch):.2f} deg")
    print()

# =========================
# 3D MODEL
# =========================

def make_board():
    length = 1.0
    width = 0.55
    height = 0.08

    x = length / 2
    y = width / 2
    z = height / 2

    vertices = np.array([
        [-x, -y, -z],
        [ x, -y, -z],
        [ x,  y, -z],
        [-x,  y, -z],

        [-x, -y,  z],
        [ x, -y,  z],
        [ x,  y,  z],
        [-x,  y,  z],
    ])

    faces = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
    ]

    return vertices, faces


def rotation_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr,  cr]
    ])

    ry = np.array([
        [ cp, 0, sp],
        [  0, 1,  0],
        [-sp, 0, cp]
    ])

    rz = np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [ 0,   0, 1]
    ])

    return rz @ ry @ rx


def rotate(points, roll, pitch, yaw):
    return points @ rotation_matrix(roll, pitch, yaw).T


# =========================
# STATE
# =========================

roll = 0.0
pitch = 0.0
yaw = 0.0

last_time = time.time()
last_packet_time = time.time()


# =========================
# PLOT
# =========================

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.set_title("ICM-20948 Live 3D Motion")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_zlim(-1.5, 1.5)
ax.set_box_aspect((1, 1, 1))

vertices, faces = make_board()

board = Poly3DCollection([], alpha=0.85, edgecolor="black")
ax.add_collection3d(board)

info = ax.text2D(0.02, 0.94, "Waiting for IMU data...", transform=ax.transAxes)

x_arrow = ax.quiver(0, 0, 0, 1, 0, 0, length=1.0)
y_arrow = ax.quiver(0, 0, 0, 0, 1, 0, length=0.8)
z_arrow = ax.quiver(0, 0, 0, 0, 0, 1, length=0.6)


# =========================
# UPDATE LOOP
# =========================

def update(_):
    global roll, pitch, yaw
    global last_time, last_packet_time
    global x_arrow, y_arrow, z_arrow

    now = time.time()
    dt = now - last_time
    last_time = now

    packet = read_latest_packet()

    if packet is not None:
        last_packet_time = now

        ax_g, ay_g, az_g, gx, gy, gz, mx, my, mz, temp_c = packet

        gx = (gx - gyro_bias_x) * MOVEMENT_SCALE
        gy = (gy - gyro_bias_y) * MOVEMENT_SCALE
        gz = (gz - gyro_bias_z) * MOVEMENT_SCALE

        # Gyro integration
        gyro_roll = roll + gx * dt
        gyro_pitch = pitch + gy * dt
        gyro_yaw = yaw + gz * dt

        # Accel tilt correction
        accel_roll = math.atan2(ay_g, az_g) - accel_zero_roll
        accel_pitch = math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)) - accel_zero_pitch

        roll = GYRO_WEIGHT * gyro_roll + (1.0 - GYRO_WEIGHT) * accel_roll
        pitch = GYRO_WEIGHT * gyro_pitch + (1.0 - GYRO_WEIGHT) * accel_pitch

        # Yaw uses gyro only unless we add magnetometer fusion
        yaw = gyro_yaw

        info.set_text(
            f"Receiving ICM-20948 data\n"
            f"Roll:  {math.degrees(roll):7.2f} deg\n"
            f"Pitch: {math.degrees(pitch):7.2f} deg\n"
            f"Yaw:   {math.degrees(yaw):7.2f} deg\n"
            f"Accel g: {ax_g:.3f}, {ay_g:.3f}, {az_g:.3f}\n"
            f"Gyro rad/s: {gx:.3f}, {gy:.3f}, {gz:.3f}\n"
            f"Temp: {temp_c:.2f} C"
        )

    else:
        if now - last_packet_time > 2:
            info.set_text(
                "No parsed IMU packets.\n"
                "Python is connected, but your serial format may not match.\n"
                "Check the terminal SERIAL lines."
            )

    rotated = rotate(vertices, roll, pitch, yaw)
    board.set_verts([[rotated[i] for i in face] for face in faces])

    x_arrow.remove()
    y_arrow.remove()
    z_arrow.remove()

    r = rotation_matrix(roll, pitch, yaw)

    x_dir = r @ np.array([1, 0, 0])
    y_dir = r @ np.array([0, 1, 0])
    z_dir = r @ np.array([0, 0, 1])

    x_arrow = ax.quiver(0, 0, 0, x_dir[0], x_dir[1], x_dir[2], length=1.0)
    y_arrow = ax.quiver(0, 0, 0, y_dir[0], y_dir[1], y_dir[2], length=0.8)
    z_arrow = ax.quiver(0, 0, 0, z_dir[0], z_dir[1], z_dir[2], length=0.6)

    return board, info

calibrate_imu()

ani = FuncAnimation(
    fig,
    update,
    interval=UPDATE_MS,
    blit=False,
    cache_frame_data=False
)

try:
    plt.show()
finally:
    ser.close()
    print("Serial closed.")
