# Core dependencies for IMU sensor fusion and serial communication
import serial
import numpy as np
from ahrs.filters import Madgwick
import time
import sys

# Serial configuration
serialPort = "COM3"
baudRate = 115200
deltaTime = 1 / 50.0  # Sample period for Madgwick filter (50 Hz)
lastPrintTime = 0.0
printFrequency = 10  # Console output rate in Hz

# Establish serial connection to IMU device
serialConnection = serial.Serial(serialPort, baudRate, timeout=1)

# Initialize Madgwick filters for each IMU unit (upper, mid, lower trunk)
madgwickFilters = {
    1: Madgwick(sampleperiod=deltaTime),
    2: Madgwick(sampleperiod=deltaTime),
    3: Madgwick(sampleperiod=deltaTime),
}

# Quaternion state for each IMU unit (w, x, y, z)
quaternions = {
    1: np.array([1.0, 0.0, 0.0, 0.0]),
    2: np.array([1.0, 0.0, 0.0, 0.0]),
    3: np.array([1.0, 0.0, 0.0, 0.0]),
}

# Timestamp tracking for sensor synchronization
lastUpdateTime = {
    1: 0.0,
    2: 0.0,
    3: 0.0,
}
syncWindow = 0.06  # Maximum allowed time difference between sensors (60 ms)


# Baseline quaternions for neutral posture reference
baselineQuaternions = {1: None, 2: None, 3: None}
baselineCaptured = False

print("NeuroCore: 3-IMU Trunk Kinematics Pipeline")

def quaternionConjugate(q):
    """Calculate quaternion conjugate for inverse rotation."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quaternionMultiply(q1, q2):
    """Multiply two quaternions for rotation composition."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quaternionToEuler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
    w, x, y, z = q
    # Calculate roll (rotation around X-axis)
    roll = np.degrees(np.arctan2(
        2*(w*x + y*z),
        1 - 2*(x*x + y*y)
    ))
    # Calculate pitch (rotation around Y-axis)
    pitch = np.degrees(np.arcsin(
        np.clip(2*(w*y - z*x), -1.0, 1.0)
    ))
    # Calculate yaw (rotation around Z-axis)
    yaw = np.degrees(np.arctan2(
        2*(w*z + x*y),
        1 - 2*(y*y + z*z)
    ))
    return roll, pitch, yaw

# Main processing loop for real-time IMU data
while True:
    rawLine = serialConnection.readline().decode(errors="ignore").strip()
    if not rawLine:
        continue

    # Parse CSV data packet
    dataParts = rawLine.split(",")
    if len(dataParts) != 8:
        continue

    # Extract unit ID and sensor readings
    unitId = int(dataParts[0])
    accelX, accelY, accelZ = map(float, dataParts[2:5])
    gyroX, gyroY, gyroZ = map(float, dataParts[5:8])

    if unitId not in madgwickFilters:
        continue

    # Update orientation estimate using Madgwick filter
    quaternions[unitId] = madgwickFilters[unitId].updateIMU(
        quaternions[unitId],
        gyr=np.array([gyroX, gyroY, gyroZ]),
        acc=np.array([accelX, accelY, accelZ])
    )

    lastUpdateTime[unitId] = time.time()

    # Check sensor synchronization before processing
    currentTime = time.time()
    if any(currentTime - lastUpdateTime[k] > syncWindow for k in lastUpdateTime):
        continue



    # Capture baseline neutral posture on first iteration
    if not baselineCaptured:
        print("\n>>> Stand neutral. Press ENTER to capture baseline.")
        input()
        for k in quaternions:
            baselineQuaternions[k] = quaternions[k].copy()
        baselineCaptured = True
        print(">>> Baseline captured\n")
        continue

    # Calculate relative orientations from baseline neutral posture
    relativeQuaternions = {}
    for k in quaternions:
        relativeQuaternions[k] = quaternionMultiply(
            quaternionConjugate(baselineQuaternions[k]),
            quaternions[k]
        )

    # Calculate inter-segment relative orientations
    thoracicQuaternion = quaternionMultiply(quaternionConjugate(relativeQuaternions[2]), relativeQuaternions[1])  # Upper vs mid
    lumbarQuaternion = quaternionMultiply(quaternionConjugate(relativeQuaternions[3]), relativeQuaternions[2])  # Mid vs lower
    globalQuaternion = quaternionMultiply(quaternionConjugate(relativeQuaternions[3]), relativeQuaternions[1])  # Upper vs lower

    # Convert quaternions to anatomical angles (degrees)
    thoracicRoll, thoracicPitch, thoracicYaw = quaternionToEuler(thoracicQuaternion)
    lumbarRoll, lumbarPitch, lumbarYaw = quaternionToEuler(lumbarQuaternion)
    globalRoll, globalPitch, globalYaw = quaternionToEuler(globalQuaternion)

    # Calculate asymmetry ratios for movement quality assessment
    flexionRatio = abs(thoracicPitch) / (abs(lumbarPitch) + 1e-6)
    lateralRatio = abs(thoracicRoll) / (abs(lumbarRoll) + 1e-6)
    rotationRatio = abs(thoracicYaw) / (abs(lumbarYaw) + 1e-6)

    # Display diagnostic output at specified frequency
    currentTime = time.time()
    if currentTime - lastPrintTime > 1.0 / printFrequency:
        lastPrintTime = currentTime
        print(
            f"Flex(T/L)={thoracicPitch:+5.1f}/{lumbarPitch:+5.1f}  "
            f"Lat(T/L)={thoracicRoll:+5.1f}/{lumbarRoll:+5.1f}  "
            f"Rot(T/L)={thoracicYaw:+5.1f}/{lumbarYaw:+5.1f}  "
            f"| Ratios F/L/R={flexionRatio:4.2f}/{lateralRatio:4.2f}/{rotationRatio:4.2f}"
        )


