import math
import re
import sys
import time
from dataclasses import dataclass

import serial
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QApplication, QFrame, QGridLayout, QLabel, QMainWindow, QWidget


# PORT = "/dev/cu.usbserial-5B1F0119041"
PORT = "COM3"
BAUD = 115200
SERIAL_TIMEOUT = 0.005
UPDATE_MS = 16
CALIBRATION_SECONDS = 5.0
MIN_CALIBRATION_SAMPLES = 20
DISPLAY_RANGE_DEG = 30.0


@dataclass
class Pose:
    back_roll: float = 0.0
    back_pitch: float = 0.0
    hip_roll: float = 0.0
    hip_pitch: float = 0.0
    relative_roll: float = 0.0
    relative_pitch: float = 0.0
    score: float = 0.0
    status: str = "WAITING"
    age_ms: int = 0


ser = None
back_zero_roll = 0.0
back_zero_pitch = 0.0
hip_zero_roll = 0.0
hip_zero_pitch = 0.0

back_roll = 0.0
back_pitch = 0.0
hip_roll = 0.0
hip_pitch = 0.0
last_packet_time = 0.0


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


def read_latest_packet():
    latest = None

    while ser and ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        packet = parse_frame(line)

        if packet:
            latest = packet

    if latest:
        return latest

    if not ser:
        return None

    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        return None

    return parse_frame(line)


def accel_roll_pitch(ax, ay, az):
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return math.degrees(roll), math.degrees(pitch)


def wait_for_data():
    print("Waiting for FRAME data...")

    while True:
        packet = read_latest_packet()

        if packet:
            print("Data detected.")
            return


def calibrate():
    global back_zero_roll, back_zero_pitch, hip_zero_roll, hip_zero_pitch

    print()
    print("CALIBRATION")
    print("Put both sensors on the brace in their real positions.")
    print("Have the brace/person in neutral upright posture.")
    print(f"Keep still for {CALIBRATION_SECONDS:.0f} seconds.")
    print()

    samples = []
    start = time.time()

    while time.time() - start < CALIBRATION_SECONDS:
        packet = read_latest_packet()

        if packet:
            samples.append(packet)
            print("Samples:", len(samples), end="\r")

        time.sleep(0.002)

    print()

    if len(samples) < MIN_CALIBRATION_SAMPLES:
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
    ser.reset_input_buffer()

    print("Calibration complete.")
    print("Back zero roll:", round(back_zero_roll, 2))
    print("Back zero pitch:", round(back_zero_pitch, 2))
    print("Hip zero roll:", round(hip_zero_roll, 2))
    print("Hip zero pitch:", round(hip_zero_pitch, 2))
    print()


def compute_pose():
    global back_roll, back_pitch, hip_roll, hip_pitch, last_packet_time

    packet = read_latest_packet()
    now = time.time()

    if packet:
        b_ax, b_ay, b_az, h_ax, h_ay, h_az = packet

        br, bp = accel_roll_pitch(b_ax, b_ay, b_az)
        hr, hp = accel_roll_pitch(h_ax, h_ay, h_az)

        back_roll = br - back_zero_roll
        back_pitch = bp - back_zero_pitch
        hip_roll = hr - hip_zero_roll
        hip_pitch = hp - hip_zero_pitch
        last_packet_time = now

    relative_roll = back_roll - hip_roll
    relative_pitch = back_pitch - hip_pitch
    score = abs(relative_roll) + abs(relative_pitch)

    if score < 8:
        status = "GOOD"
    elif score < 15:
        status = "WATCH"
    else:
        status = "HAPTIC FEEDBACK"

    age_ms = int(max(0.0, now - last_packet_time) * 1000) if last_packet_time else 0

    return Pose(
        back_roll=back_roll,
        back_pitch=back_pitch,
        hip_roll=hip_roll,
        hip_pitch=hip_pitch,
        relative_roll=relative_roll,
        relative_pitch=relative_pitch,
        score=score,
        status=status,
        age_ms=age_ms,
    )


class MetricCard(QFrame):
    def __init__(self, title, unit="deg"):
        super().__init__()
        self.unit = unit
        self.setObjectName("metricCard")

        layout = QGridLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setVerticalSpacing(2)

        self.title_label = QLabel(title)
        self.title_label.setObjectName("metricTitle")

        self.value_label = QLabel("0.0")
        self.value_label.setObjectName("metricValue")
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("metricUnit")
        self.unit_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(self.title_label, 0, 0, 1, 2)
        layout.addWidget(self.value_label, 1, 0)
        layout.addWidget(self.unit_label, 1, 1)

    def set_value(self, value):
        self.value_label.setText(f"{value:.1f}")


class LeanGauge(QWidget):
    def __init__(self):
        super().__init__()
        self._side = 0.0
        self._forward = 0.0
        self.setMinimumHeight(240)

    def set_pose(self, side, forward):
        self._side = max(-DISPLAY_RANGE_DEG, min(DISPLAY_RANGE_DEG, side))
        self._forward = max(-DISPLAY_RANGE_DEG, min(DISPLAY_RANGE_DEG, forward))
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(18, 16, -18, -16)
        center = rect.center()
        radius = min(rect.width(), rect.height()) / 2

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111827"))
        painter.drawEllipse(center, radius, radius)

        painter.setPen(QPen(QColor("#263244"), 1))
        for scale in (0.33, 0.66, 1.0):
            r = radius * scale
            painter.drawEllipse(center, r, r)

        painter.setPen(QPen(QColor("#3a465a"), 1))
        painter.drawLine(center.x() - radius, center.y(), center.x() + radius, center.y())
        painter.drawLine(center.x(), center.y() - radius, center.x(), center.y() + radius)

        x = center.x() + (self._side / DISPLAY_RANGE_DEG) * radius
        y = center.y() - (self._forward / DISPLAY_RANGE_DEG) * radius

        color = status_color(abs(self._side) + abs(self._forward))
        painter.setBrush(QColor(color))
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawEllipse(int(x - 10), int(y - 10), 20, 20)

        painter.setPen(QPen(QColor("#94a3b8"), 1))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(rect.adjusted(0, 0, 0, -6), Qt.AlignBottom | Qt.AlignHCenter, "side lean")
        painter.save()
        painter.translate(rect.left() + 8, center.y())
        painter.rotate(-90)
        painter.drawText(-70, 0, "forward lean")
        painter.restore()


class BarMeter(QWidget):
    def __init__(self, label):
        super().__init__()
        self.label = label
        self.value = 0.0
        self.setMinimumHeight(30)

    def set_value(self, value):
        self.value = max(-DISPLAY_RANGE_DEG, min(DISPLAY_RANGE_DEG, value))
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(0, 7, -48, -7)
        center_x = rect.left() + rect.width() / 2
        fill_width = abs(self.value) / DISPLAY_RANGE_DEG * (rect.width() / 2)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111827"))
        painter.drawRoundedRect(rect, 5, 5)

        painter.setBrush(QColor(status_color(abs(self.value))))
        if self.value >= 0:
            fill_rect = rect.adjusted(int(rect.width() / 2), 0, 0, 0)
            fill_rect.setWidth(int(fill_width))
        else:
            fill_rect = rect.adjusted(0, 0, -int(rect.width() / 2), 0)
            fill_rect.setLeft(int(center_x - fill_width))

        painter.drawRoundedRect(fill_rect, 5, 5)
        painter.setPen(QPen(QColor("#526075"), 1))
        painter.drawLine(int(center_x), rect.top(), int(center_x), rect.bottom())

        painter.setPen(QColor("#e5e7eb"))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(0, 0, rect.width(), 11, Qt.AlignLeft | Qt.AlignTop, self.label)

        painter.setPen(QColor("#cbd5e1"))
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.drawText(self.rect().adjusted(0, 0, -2, 0), Qt.AlignRight | Qt.AlignVCenter, f"{self.value:+.1f}")


class ScoreRing(QWidget):
    def __init__(self):
        super().__init__()
        self._score = 0.0
        self.setMinimumSize(140, 140)

    def set_score(self, score):
        self._score = score
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(14, 14, -14, -14)
        score_clamped = min(24.0, self._score)

        painter.setPen(QPen(QColor("#172033"), 12, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(rect, 90 * 16, -360 * 16)

        painter.setPen(QPen(QColor(status_color(self._score)), 12, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(rect, 90 * 16, int(-360 * 16 * (score_clamped / 24.0)))

        painter.setPen(QColor("#f8fafc"))
        painter.setFont(QFont("Segoe UI", 26, QFont.Bold))
        painter.drawText(rect, Qt.AlignCenter, f"{self._score:.0f}")

        painter.setPen(QColor("#94a3b8"))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(rect.adjusted(0, 54, 0, 0), Qt.AlignCenter, "score")


class StatusPill(QLabel):
    def __init__(self):
        super().__init__("WAITING")
        self.setAlignment(Qt.AlignCenter)
        self.setObjectName("statusPill")

    def set_status(self, status, score):
        self.setText(status)
        self.setStyleSheet(
            f"""
            QLabel#statusPill {{
                background: {status_color(score)};
                color: #07111f;
                border-radius: 14px;
                padding: 6px 14px;
                font: 800 12px "Segoe UI";
            }}
            """
        )


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NeuroCore Live Brace Monitor")
        self.resize(960, 600)
        self.setMinimumSize(820, 520)

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)

        layout = QGridLayout(root)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)

        title = QLabel("NeuroCore Live Brace Monitor")
        title.setObjectName("title")

        self.status = StatusPill()
        self.age_label = QLabel("sensor age 0 ms")
        self.age_label.setObjectName("ageLabel")
        self.age_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(title, 0, 0, 1, 2)
        layout.addWidget(self.status, 0, 2)
        layout.addWidget(self.age_label, 0, 3)

        self.gauge = LeanGauge()
        layout.addWidget(self.gauge, 1, 0, 5, 2)

        self.score_ring = ScoreRing()
        layout.addWidget(self.score_ring, 1, 2, 2, 2)

        self.relative_side = MetricCard("Relative side lean")
        self.relative_forward = MetricCard("Relative forward lean")
        layout.addWidget(self.relative_side, 3, 2)
        layout.addWidget(self.relative_forward, 3, 3)

        self.meters = [
            BarMeter("Back side lean"),
            BarMeter("Hip side tilt"),
            BarMeter("Relative side lean"),
            BarMeter("Back forward lean"),
            BarMeter("Hip forward tilt"),
            BarMeter("Relative forward lean"),
        ]

        for index, meter in enumerate(self.meters):
            layout.addWidget(meter, 6 + index, 0, 1, 4)

        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 2)
        layout.setColumnStretch(2, 1)
        layout.setColumnStretch(3, 1)
        layout.setRowStretch(5, 0)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(UPDATE_MS)

    def refresh(self):
        pose = compute_pose()
        values = [
            pose.back_roll,
            pose.hip_roll,
            pose.relative_roll,
            pose.back_pitch,
            pose.hip_pitch,
            pose.relative_pitch,
        ]

        for meter, value in zip(self.meters, values):
            meter.set_value(value)

        self.gauge.set_pose(pose.relative_roll, pose.relative_pitch)
        self.score_ring.set_score(pose.score)
        self.relative_side.set_value(pose.relative_roll)
        self.relative_forward.set_value(pose.relative_pitch)
        self.status.set_status(pose.status, pose.score)
        self.age_label.setText(f"sensor age {pose.age_ms} ms")


def status_color(score):
    if score < 8:
        return "#22c55e"
    if score < 15:
        return "#facc15"
    return "#fb7185"


def set_style(app):
    app.setStyleSheet(
        """
        QWidget#root {
            background: #0b1020;
            color: #e5e7eb;
        }

        QLabel#title {
            color: #f8fafc;
            font: 800 20px "Segoe UI";
        }

        QLabel#ageLabel {
            color: #94a3b8;
            font: 600 11px "Segoe UI";
        }

        QFrame#metricCard {
            background: #111827;
            border: 1px solid #253044;
            border-radius: 8px;
        }

        QLabel#metricTitle {
            color: #94a3b8;
            font: 600 11px "Segoe UI";
        }

        QLabel#metricValue {
            color: #f8fafc;
            font: 800 28px "Segoe UI";
        }

        QLabel#metricUnit {
            color: #64748b;
            font: 700 11px "Segoe UI";
        }
        """
    )


def main():
    global ser

    ser = serial.Serial(PORT, BAUD, timeout=SERIAL_TIMEOUT)
    time.sleep(3)
    ser.reset_input_buffer()

    wait_for_data()
    calibrate()

    app = QApplication(sys.argv)
    set_style(app)

    dashboard = Dashboard()
    dashboard.show()

    try:
        return app.exec()
    finally:
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    raise SystemExit(main())
