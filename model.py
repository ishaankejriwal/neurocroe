import serial, time, re, math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = "/dev/cu.usbserial-5B1F0119041"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(3)
ser.reset_input_buffer()

back_zero_roll = 0.0
back_zero_pitch = 0.0
hip_zero_roll = 0.0
hip_zero_pitch = 0.0

back_roll = 0.0
back_pitch = 0.0
hip_roll = 0.0
hip_pitch = 0.0


def parse_frame(line):
    if "FRAME BACK" not in line or "| HIP" not in line:
        return None

    nums = re.findall(r"[-+]?\d+\.\d+|[-+]?\d+", line)

    if len(nums) < 20:
        return None

    b_ax = float(nums[0]) / 1000.0
    b_ay = float(nums[1]) / 1000.0
    b_az = float(nums[2]) / 1000.0

    h_ax = float(nums[10]) / 1000.0
    h_ay = float(nums[11]) / 1000.0
    h_az = float(nums[12]) / 1000.0

    return b_ax, b_ay, b_az, h_ax, h_ay, h_az


def read_packet():
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            return None

        packet = parse_frame(line)
        if packet:
            return packet


def accel_roll_pitch(ax, ay, az):
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return math.degrees(roll), math.degrees(pitch)


def wait_for_data():
    print("Waiting for FRAME data...")
    while True:
        packet = read_packet()
        if packet:
            print("Data detected.")
            return


def calibrate():
    global back_zero_roll, back_zero_pitch, hip_zero_roll, hip_zero_pitch

    print()
    print("CALIBRATION")
    print("Put both sensors on the brace in their real positions.")
    print("Have the brace/person in neutral upright posture.")
    print("Keep still for 5 seconds.")
    print()

    samples = []
    start = time.time()

    while time.time() - start < 5:
        packet = read_packet()
        if packet:
            samples.append(packet)
            print("Samples:", len(samples), end="\r")

    print()

    if len(samples) < 20:
        print("Not enough samples. Try again.")
        return

    b_rolls = []
    b_pitches = []
    h_rolls = []
    h_pitches = []

    for b_ax, b_ay, b_az, h_ax, h_ay, h_az in samples:
        br, bp = accel_roll_pitch(b_ax, b_ay, b_az)
        hr, hp = accel_roll_pitch(h_ax, h_ay, h_az)

        b_rolls.append(br)
        b_pitches.append(bp)
        h_rolls.append(hr)
        h_pitches.append(hp)

    back_zero_roll = sum(b_rolls) / len(b_rolls)
    back_zero_pitch = sum(b_pitches) / len(b_pitches)
    hip_zero_roll = sum(h_rolls) / len(h_rolls)
    hip_zero_pitch = sum(h_pitches) / len(h_pitches)

    print("Calibration complete.")
    print("Back zero roll:", round(back_zero_roll, 2))
    print("Back zero pitch:", round(back_zero_pitch, 2))
    print("Hip zero roll:", round(hip_zero_roll, 2))
    print("Hip zero pitch:", round(hip_zero_pitch, 2))
    print()


wait_for_data()
calibrate()

fig, ax = plt.subplots()
ax.set_title("Two-IMU Diagnostic: Back vs Hip")
ax.set_xlim(-30, 30)
ax.set_ylim(-1, 6)
ax.axvline(0, linewidth=1)

labels = [
    "Back side lean",
    "Hip side tilt",
    "Relative side lean",
    "Back forward lean",
    "Hip forward tilt",
    "Relative forward lean",
]

bars = ax.barh(range(len(labels)), [0] * len(labels))
ax.set_yticks(range(len(labels)))
ax.set_yticklabels(labels)
ax.set_xlabel("Degrees")

text = ax.text(0.02, 0.95, "", transform=ax.transAxes, va="top")


def update(_):
    global back_roll, back_pitch, hip_roll, hip_pitch

    packet = read_packet()

    if packet:
        b_ax, b_ay, b_az, h_ax, h_ay, h_az = packet

        br, bp = accel_roll_pitch(b_ax, b_ay, b_az)
        hr, hp = accel_roll_pitch(h_ax, h_ay, h_az)

        back_roll = br - back_zero_roll
        back_pitch = bp - back_zero_pitch

        hip_roll = hr - hip_zero_roll
        hip_pitch = hp - hip_zero_pitch

    relative_roll = back_roll - hip_roll
    relative_pitch = back_pitch - hip_pitch

    values = [
        back_roll,
        hip_roll,
        relative_roll,
        back_pitch,
        hip_pitch,
        relative_pitch,
    ]

    for bar, value in zip(bars, values):
        bar.set_width(value)

    score = abs(relative_roll) + abs(relative_pitch)

    if score < 8:
        status = "GOOD"
    elif score < 15:
        status = "WATCH"
    else:
        status = "HAPTIC FEEDBACK"

    text.set_text(
        f"Relative side lean: {relative_roll:.1f} deg\n"
        f"Relative forward lean: {relative_pitch:.1f} deg\n"
        f"Compensation score: {score:.1f}\n"
        f"Status: {status}"
    )

    return list(bars) + [text]


ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

try:
    plt.show()
finally:
    ser.close()
    print("Serial closed.")
PY

python3 two_imu_diagnostic.py


