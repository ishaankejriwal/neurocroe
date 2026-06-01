# Force matplotlib to use TkAgg backend for compatibility
import matplotlib
matplotlib.use("TkAgg")

# Core dependencies for serial communication and 3D visualization
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Serial configuration
serialPort = "COM3"  # USB serial port for IMU device
baudRate = 115200  # Communication speed in bits per second
deltaTime = 1 / 50.0  # Time step for gyroscope integration (50 Hz)
targetUnit = 3  # IMU unit ID to filter data from
rotationScale = 8.0  # Amplification factor for visualization

# Establish serial connection and wait for device initialization
serialConnection = serial.Serial(serialPort, baudRate, timeout=0.01)
time.sleep(2)

# Orientation state (roll, pitch, yaw)
eulerAngles = np.zeros(3)

def createBox(size=(0.6, 0.35, 0.25)):
    """Generate 3D box vertices."""
    x, y, z = size
    # Define 8 vertices of a rectangular prism centered at origin
    return np.array([
        [-x, -y, -z], [ x, -y, -z], [ x,  y, -z], [-x,  y, -z],
        [-x, -y,  z], [ x, -y,  z], [ x,  y,  z], [-x,  y,  z]
    ])

# Define box face indices for 3D rendering (front, back, sides)
boxFaces = [
    [0, 1, 2, 3], [4, 5, 6, 7],
    [0, 1, 5, 4], [2, 3, 7, 6],
    [1, 2, 6, 5], [0, 3, 7, 4]
]

def applyRotation(points, roll, pitch, yaw):
    """Apply Euler angle rotation to 3D points."""
    # Create rotation matrices for each axis (X=roll, Y=pitch, Z=yaw)
    rotX = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    rotY = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rotZ = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    # Apply composite rotation: Z * Y * X (yaw, pitch, roll order)
    return points @ (rotZ @ rotY @ rotX).T

# Initialize 3D plot figure and axes
plotFigure = plt.figure()
plotAxis = plotFigure.add_subplot(111, projection="3d")
# Set consistent axis limits for stable visualization
plotAxis.set_xlim(-1.5, 1.5)
plotAxis.set_ylim(-1.5, 1.5)
plotAxis.set_zlim(-1.5, 1.5)
plotAxis.set_box_aspect((1, 1, 1))  # Equal aspect ratio for all axes
plotAxis.set_title("NeuroCore Live Visualization")

# Create box geometry and add to scene
boxVertices = createBox()
boxPoly = Poly3DCollection([], alpha=0.8, facecolor="tab:green")
plotAxis.add_collection3d(boxPoly)

def updateFrame(_):
    """Update animation frame with latest IMU data."""
    global eulerAngles

    # Process all available serial data in buffer
    while serialConnection.in_waiting:
        rawLine = serialConnection.readline().decode(errors="ignore").strip()
        dataParts = rawLine.split(",")
        # Validate data format (8 comma-separated values expected)
        if len(dataParts) != 8:
            continue

        # Filter data from target IMU unit only
        if int(dataParts[0]) != targetUnit:
            continue

        # Extract gyroscope readings from data packet (indices 5-7)
        gyroX, gyroY, gyroZ = map(float, dataParts[5:8])

        # Integrate gyroscope data to update Euler angles
        eulerAngles[0] += gyroX * deltaTime * rotationScale
        eulerAngles[1] += gyroY * deltaTime * rotationScale
        eulerAngles[2] += gyroZ * deltaTime * rotationScale

    # Apply current orientation to box geometry
    roll, pitch, yaw = eulerAngles
    rotatedPoints = applyRotation(boxVertices, roll, pitch, yaw)
    boxPoly.set_verts([[rotatedPoints[i] for i in face] for face in boxFaces])

# Start animation loop with 20ms frame interval (50 FPS)
animation = FuncAnimation(plotFigure, updateFrame, interval=20)
plt.show()
