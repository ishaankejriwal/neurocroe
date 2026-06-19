import csv
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QApplication, QComboBox, QFrame, QGridLayout, QLabel, QMainWindow, QPushButton, QWidget


# Serial and dashboard configuration.
# PORT = "/dev/cu.usbserial-5B1F0119041"
PORT = "COM3"
BAUD = 115200
SERIAL_TIMEOUT = 0.005
UPDATE_MS = 16
CALIBRATION_SECONDS = 5.0
MIN_CALIBRATION_SAMPLES = 20
DISPLAY_RANGE_DEG = 30.0
TRIAL_LOG_PATH = Path("trunk_stability_trials.csv")
STABLE_RECOVERY_DEG = 3.0
STABLE_RECOVERY_SECONDS = 0.5

# Task-specific thresholds keep easier and harder exercises clinically separate.
TASK_THRESHOLDS = {
    "Quiet standing - 30 s": 6.0,
    "Feet together standing - 30 s": 8.0,
    "Seated unsupported hold - 30 s": 8.0,
    "Seated leg raise - left/right": 10.0,
    "Sit-to-stand - 5 reps": 12.0,
    "Standing march - 10 reps": 12.0,
}

# Side labels support symmetric tasks and left/right exercise comparisons.
SIDE_OPTIONS = ["Both / none", "Left", "Right"]


# Stores the current calibrated brace posture for the dashboard.
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


# One time-stamped posture sample recorded during an assessment trial.
@dataclass
class TrialSample:
    elapsed_s: float
    relative_roll: float
    relative_pitch: float
    relative_mag: float
    score: float


# Summary metrics computed from a completed exercise trial.
@dataclass
class TrialMetrics:
    duration_s: float = 0.0
    sample_count: int = 0
    rms_deg: float = 0.0
    p95_deg: float = 0.0
    peak_deg: float = 0.0
    outside_pct: float = 0.0
    path_deg: float = 0.0
    velocity_deg_s: float = 0.0
    recovery_s: float | None = None


# Runtime serial connection; initialized in main so imports stay safe.
ser = None

# Neutral calibration offsets captured while the brace is upright.
back_zero_roll = 0.0
back_zero_pitch = 0.0
hip_zero_roll = 0.0
hip_zero_pitch = 0.0

# Last known posture values are retained when no new packet arrives.
back_roll = 0.0
back_pitch = 0.0
hip_roll = 0.0
hip_pitch = 0.0
last_packet_time = 0.0


def parse_frame(line):
    # Ignore any serial output that is not the expected two-IMU frame.
    if "FRAME BACK" not in line or "| HIP" not in line:
        return None

    # Extract numeric fields from the firmware text format.
    nums = re.findall(r"[-+]?\d+\.\d+|[-+]?\d+", line)

    # A complete frame contains back and hip accelerometer/gyro/mag fields.
    if len(nums) < 20:
        return None

    # Convert back accelerometer readings from milli-g to g.
    b_ax = float(nums[0]) / 1000.0
    b_ay = float(nums[1]) / 1000.0
    b_az = float(nums[2]) / 1000.0

    # Convert hip accelerometer readings from milli-g to g.
    h_ax = float(nums[10]) / 1000.0
    h_ay = float(nums[11]) / 1000.0
    h_az = float(nums[12]) / 1000.0

    return b_ax, b_ay, b_az, h_ax, h_ay, h_az


def read_latest_packet():
    # Drain the serial backlog so the UI always uses the newest valid frame.
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

    # If no backlog exists, do one short blocking read for fresh data.
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        return None

    return parse_frame(line)


def accel_roll_pitch(ax, ay, az):
    # Estimate roll and pitch from acceleration relative to gravity.
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return math.degrees(roll), math.degrees(pitch)


def wait_for_data():
    # Block startup until the firmware is producing parseable frames.
    print("Waiting for FRAME data...")

    while True:
        packet = read_latest_packet()

        if packet:
            print("Data detected.")
            return


def calibrate():
    # Capture upright baseline angles so live readings become relative posture.
    global back_zero_roll, back_zero_pitch, hip_zero_roll, hip_zero_pitch

    print()
    print("CALIBRATION")
    print("Put both sensors on the brace in their real positions.")
    print("Have the brace/person in neutral upright posture.")
    print(f"Keep still for {CALIBRATION_SECONDS:.0f} seconds.")
    print()

    samples = []
    start = time.time()

    # Collect a short burst of samples while the brace is held still.
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

    # Convert all calibration samples into roll/pitch angle lists.
    for b_ax, b_ay, b_az, h_ax, h_ay, h_az in samples:
        br, bp = accel_roll_pitch(b_ax, b_ay, b_az)
        hr, hp = accel_roll_pitch(h_ax, h_ay, h_az)

        b_rolls.append(br)
        b_pitches.append(bp)
        h_rolls.append(hr)
        h_pitches.append(hp)

    # Average calibration angles to reduce noise in the neutral baseline.
    back_zero_roll = sum(b_rolls) / len(b_rolls)
    back_zero_pitch = sum(b_pitches) / len(b_pitches)
    hip_zero_roll = sum(h_rolls) / len(h_rolls)
    hip_zero_pitch = sum(h_pitches) / len(h_pitches)

    # Clear any calibration-era packets before the live dashboard starts.
    ser.reset_input_buffer()

    print("Calibration complete.")
    print("Back zero roll:", round(back_zero_roll, 2))
    print("Back zero pitch:", round(back_zero_pitch, 2))
    print("Hip zero roll:", round(hip_zero_roll, 2))
    print("Hip zero pitch:", round(hip_zero_pitch, 2))
    print()


def compute_pose():
    # Update posture state from the newest packet and derive compensation metrics.
    global back_roll, back_pitch, hip_roll, hip_pitch, last_packet_time

    packet = read_latest_packet()
    now = time.time()

    if packet:
        b_ax, b_ay, b_az, h_ax, h_ay, h_az = packet

        br, bp = accel_roll_pitch(b_ax, b_ay, b_az)
        hr, hp = accel_roll_pitch(h_ax, h_ay, h_az)

        # Subtract neutral offsets so displayed values represent deviation.
        back_roll = br - back_zero_roll
        back_pitch = bp - back_zero_pitch
        hip_roll = hr - hip_zero_roll
        hip_pitch = hp - hip_zero_pitch
        last_packet_time = now

    # Relative posture is the back movement after removing hip movement.
    relative_roll = back_roll - hip_roll
    relative_pitch = back_pitch - hip_pitch
    score = abs(relative_roll) + abs(relative_pitch)

    # Convert the compensation score into a simple clinical status band.
    if score < 8:
        status = "GOOD"
    elif score < 15:
        status = "WATCH"
    else:
        status = "HAPTIC FEEDBACK"

    # Sensor age helps diagnose connection stalls or packet delays.
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


def percentile(values, pct):
    # Compute a percentile without requiring NumPy for this lightweight script.
    if not values:
        return 0.0

    sorted_values = sorted(values)
    index = (len(sorted_values) - 1) * pct / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)

    if lower == upper:
        return sorted_values[int(index)]

    lower_weight = upper - index
    upper_weight = index - lower
    return sorted_values[lower] * lower_weight + sorted_values[upper] * upper_weight


def recovery_time_after_peak(samples):
    # Measure how long it takes to return near neutral after the largest deviation.
    if not samples:
        return None

    peak_index = max(range(len(samples)), key=lambda i: samples[i].relative_mag)
    peak_time = samples[peak_index].elapsed_s
    stable_start = None

    for sample in samples[peak_index:]:
        if sample.relative_mag <= STABLE_RECOVERY_DEG:
            if stable_start is None:
                stable_start = sample.elapsed_s

            if sample.elapsed_s - stable_start >= STABLE_RECOVERY_SECONDS:
                return max(0.0, stable_start - peak_time)
        else:
            stable_start = None

    return None


def calculate_trial_metrics(samples, threshold_deg):
    # Convert trial samples into objective trunk-over-pelvis control metrics.
    if not samples:
        return TrialMetrics()

    duration = samples[-1].elapsed_s
    magnitudes = [sample.relative_mag for sample in samples]
    rms = math.sqrt(sum(value * value for value in magnitudes) / len(magnitudes))
    p95 = percentile(magnitudes, 95)
    peak = max(magnitudes)

    outside_seconds = 0.0
    path = 0.0

    for previous, current in zip(samples, samples[1:]):
        dt = max(0.0, current.elapsed_s - previous.elapsed_s)

        if current.relative_mag > threshold_deg:
            outside_seconds += dt

        path += math.hypot(
            current.relative_roll - previous.relative_roll,
            current.relative_pitch - previous.relative_pitch,
        )

    outside_pct = (outside_seconds / duration * 100.0) if duration > 0 else 0.0
    velocity = (path / duration) if duration > 0 else 0.0

    return TrialMetrics(
        duration_s=duration,
        sample_count=len(samples),
        rms_deg=rms,
        p95_deg=p95,
        peak_deg=peak,
        outside_pct=outside_pct,
        path_deg=path,
        velocity_deg_s=velocity,
        recovery_s=recovery_time_after_peak(samples),
    )


def append_trial_csv(task, side, threshold_deg, metrics):
    # Persist completed trials so patient progress can be tracked over time.
    fieldnames = [
        "timestamp",
        "task",
        "side",
        "threshold_deg",
        "duration_s",
        "sample_count",
        "rms_deg",
        "p95_deg",
        "peak_deg",
        "outside_pct",
        "path_deg",
        "velocity_deg_s",
        "recovery_s",
    ]
    file_exists = TRIAL_LOG_PATH.exists()

    with TRIAL_LOG_PATH.open("a", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        writer.writerow(
            {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "task": task,
                "side": side,
                "threshold_deg": f"{threshold_deg:.1f}",
                "duration_s": f"{metrics.duration_s:.3f}",
                "sample_count": metrics.sample_count,
                "rms_deg": f"{metrics.rms_deg:.3f}",
                "p95_deg": f"{metrics.p95_deg:.3f}",
                "peak_deg": f"{metrics.peak_deg:.3f}",
                "outside_pct": f"{metrics.outside_pct:.3f}",
                "path_deg": f"{metrics.path_deg:.3f}",
                "velocity_deg_s": f"{metrics.velocity_deg_s:.3f}",
                "recovery_s": "" if metrics.recovery_s is None else f"{metrics.recovery_s:.3f}",
            }
        )


class MetricCard(QFrame):
    # Compact card for one large numeric dashboard value.
    def __init__(self, title, unit="deg"):
        super().__init__()
        self.unit = unit
        self.setObjectName("metricCard")
        self.setMinimumHeight(104)

        layout = QGridLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(2)

        self.title_label = QLabel(title)
        self.title_label.setObjectName("metricTitle")

        self.value_label = QLabel("0.0")
        self.value_label.setObjectName("metricValue")
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("metricUnit")
        self.unit_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.unit_label.setMinimumWidth(34)

        layout.addWidget(self.title_label, 0, 0, 1, 2)
        layout.addWidget(self.value_label, 1, 0)
        layout.addWidget(self.unit_label, 1, 1)
        layout.setColumnStretch(0, 1)

    def set_value(self, value):
        # Keep numeric formatting consistent across live updates.
        self.value_label.setText(f"{value:.1f}")


class MiniMetric(QFrame):
    # Small assessment metric tile for trial summaries.
    def __init__(self, title, value="--"):
        super().__init__()
        self.setObjectName("miniMetric")
        self.setMinimumSize(112, 42)

        layout = QGridLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setVerticalSpacing(1)

        self.title_label = QLabel(title)
        self.title_label.setObjectName("miniTitle")

        self.value_label = QLabel(value)
        self.value_label.setObjectName("miniValue")
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(self.title_label, 0, 0)
        layout.addWidget(self.value_label, 1, 0)
        layout.setColumnStretch(0, 1)

    def set_text(self, value):
        # Accept preformatted strings because assessment units vary by metric.
        self.value_label.setText(value)


class LeanGauge(QWidget):
    # Circular two-axis indicator for relative side and forward lean.
    def __init__(self):
        super().__init__()
        self._side = 0.0
        self._forward = 0.0
        self.setMinimumHeight(240)

    def set_pose(self, side, forward):
        # Clamp values so the marker always remains inside the gauge.
        self._side = max(-DISPLAY_RANGE_DEG, min(DISPLAY_RANGE_DEG, side))
        self._forward = max(-DISPLAY_RANGE_DEG, min(DISPLAY_RANGE_DEG, forward))
        self.update()

    def paintEvent(self, _event):
        # Custom paint keeps the live gauge lightweight and fast.
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(18, 16, -18, -16)
        center = rect.center()
        radius = min(rect.width(), rect.height()) / 2

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111827"))
        painter.drawEllipse(center, radius, radius)

        # Draw reference rings for quick visual magnitude estimation.
        painter.setPen(QPen(QColor("#263244"), 1))
        for scale in (0.33, 0.66, 1.0):
            r = radius * scale
            painter.drawEllipse(center, r, r)

        # Draw center axes for side lean and forward lean.
        painter.setPen(QPen(QColor("#3a465a"), 1))
        painter.drawLine(center.x() - radius, center.y(), center.x() + radius, center.y())
        painter.drawLine(center.x(), center.y() - radius, center.x(), center.y() + radius)

        # Map degrees into gauge coordinates.
        x = center.x() + (self._side / DISPLAY_RANGE_DEG) * radius
        y = center.y() - (self._forward / DISPLAY_RANGE_DEG) * radius

        # Color the marker based on the same score thresholds as the status.
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
    # Horizontal signed bar for individual angle channels.
    def __init__(self, label):
        super().__init__()
        self.label = label
        self.value = 0.0
        self.setMinimumHeight(36)

    def set_value(self, value):
        # Clamp displayed values to the configured dashboard range.
        self.value = max(-DISPLAY_RANGE_DEG, min(DISPLAY_RANGE_DEG, value))
        self.update()

    def paintEvent(self, _event):
        # Render a compact zero-centered bar without Matplotlib overhead.
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        label_width = 158
        value_width = 76
        rect = self.rect().adjusted(label_width, 9, -value_width, -9)
        center_x = rect.left() + rect.width() / 2
        fill_width = abs(self.value) / DISPLAY_RANGE_DEG * (rect.width() / 2)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111827"))
        painter.drawRoundedRect(rect, 5, 5)

        # Fill right for positive values and left for negative values.
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
        painter.drawText(0, 0, label_width - 10, self.height(), Qt.AlignLeft | Qt.AlignVCenter, self.label)

        painter.setPen(QColor("#cbd5e1"))
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.drawText(
            self.rect().adjusted(0, 0, -14, 0),
            Qt.AlignRight | Qt.AlignVCenter,
            f"{self.value:+.1f}",
        )


class ScoreRing(QWidget):
    # Circular summary indicator for the total compensation score.
    def __init__(self):
        super().__init__()
        self._score = 0.0
        self.setMinimumSize(140, 140)

    def set_score(self, score):
        # Store score and schedule a repaint on the next Qt frame.
        self._score = score
        self.update()

    def paintEvent(self, _event):
        # Draw the score ring directly for smooth real-time updates.
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(14, 14, -14, -14)
        score_clamped = min(24.0, self._score)

        # Draw the inactive ring background.
        painter.setPen(QPen(QColor("#172033"), 12, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(rect, 90 * 16, -360 * 16)

        # Draw the active score arc using the current threshold color.
        painter.setPen(QPen(QColor(status_color(self._score)), 12, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(rect, 90 * 16, int(-360 * 16 * (score_clamped / 24.0)))

        painter.setPen(QColor("#f8fafc"))
        painter.setFont(QFont("Segoe UI", 26, QFont.Bold))
        painter.drawText(rect, Qt.AlignCenter, f"{self._score:.0f}")

        painter.setPen(QColor("#94a3b8"))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(rect.adjusted(0, 54, 0, 0), Qt.AlignCenter, "score")


class StatusPill(QLabel):
    # Colored label that summarizes the current compensation state.
    def __init__(self):
        super().__init__("WAITING")
        self.setAlignment(Qt.AlignCenter)
        self.setObjectName("statusPill")

    def set_status(self, status, score):
        # Update both text and color whenever the score band changes.
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
    # Main Qt window that arranges all live monitoring widgets.
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NeuroCore Live Brace Monitor")
        self.resize(1180, 700)
        self.setMinimumSize(980, 620)
        self.trial_active = False
        self.trial_start_time = 0.0
        self.trial_samples = []
        self.side_results = {}

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)

        # Use a dense grid so the dashboard fits comfortably on a desktop screen.
        layout = QGridLayout(root)
        layout.setContentsMargins(24, 18, 24, 18)
        layout.setHorizontalSpacing(14)
        layout.setVerticalSpacing(10)

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

        self.assessment_button = QPushButton("Open Assessment Trial")
        self.assessment_button.setObjectName("secondaryButton")
        self.assessment_button.clicked.connect(self.show_assessment_window)
        layout.addWidget(self.assessment_button, 5, 2, 1, 2)

        self.assessment_window = self.create_assessment_window()

        # Individual channel meters mirror the original diagnostic bar chart.
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
        layout.setColumnMinimumWidth(2, 260)
        layout.setColumnMinimumWidth(3, 260)
        layout.setRowStretch(5, 1)

        # The Qt timer drives live updates without blocking the UI thread.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(UPDATE_MS)

    def create_assessment_window(self):
        # Keep assessment controls separate so the live monitor remains readable.
        window = QWidget(self, Qt.Window)
        window.setObjectName("assessmentRoot")
        window.setWindowTitle("NeuroCore Assessment Trial")
        window.resize(760, 320)
        window.setMinimumSize(700, 300)

        layout = QGridLayout(window)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.addWidget(self.create_assessment_panel(), 0, 0)

        return window

    def show_assessment_window(self):
        # Reuse the same trial widgets each time the user opens the panel.
        self.assessment_window.show()
        self.assessment_window.raise_()
        self.assessment_window.activateWindow()

    def create_assessment_panel(self):
        # Build task controls and metric tiles for standardized exercise trials.
        panel = QFrame()
        panel.setObjectName("assessmentPanel")
        panel.setMinimumHeight(260)

        layout = QGridLayout(panel)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(8)

        title = QLabel("Assessment Trial")
        title.setObjectName("panelTitle")

        self.task_select = QComboBox()
        self.task_select.addItems(list(TASK_THRESHOLDS))
        self.task_select.currentTextChanged.connect(lambda _text: self.update_assessment_context())
        self.task_select.setMinimumWidth(250)

        self.side_select = QComboBox()
        self.side_select.addItems(SIDE_OPTIONS)
        self.side_select.currentTextChanged.connect(lambda _text: self.update_asymmetry_display())
        self.side_select.setMinimumWidth(126)

        self.threshold_label = QLabel()
        self.threshold_label.setObjectName("thresholdLabel")
        self.threshold_label.setMinimumWidth(58)

        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_trial)
        self.stop_button.clicked.connect(self.stop_trial)

        self.trial_state_label = QLabel("ready")
        self.trial_state_label.setObjectName("trialState")
        self.trial_state_label.setMinimumWidth(190)

        self.trial_rms_metric = MiniMetric("RMS", "--")
        self.trial_p95_metric = MiniMetric("P95", "--")
        self.trial_peak_metric = MiniMetric("Peak", "--")
        self.trial_outside_metric = MiniMetric("Outside", "--")
        self.trial_velocity_metric = MiniMetric("Velocity", "--")
        self.trial_recovery_metric = MiniMetric("Recovery", "--")
        self.trial_asymmetry_metric = MiniMetric("Asymmetry", "--")
        self.trial_samples_metric = MiniMetric("Samples", "0")

        layout.addWidget(title, 0, 0, 1, 4)
        layout.addWidget(self.task_select, 1, 0, 1, 2)
        layout.addWidget(self.side_select, 1, 2)
        layout.addWidget(self.threshold_label, 1, 3)
        layout.addWidget(self.start_button, 2, 0)
        layout.addWidget(self.stop_button, 2, 1)
        layout.addWidget(self.trial_state_label, 2, 2, 1, 2)

        metrics = [
            self.trial_rms_metric,
            self.trial_p95_metric,
            self.trial_peak_metric,
            self.trial_outside_metric,
            self.trial_velocity_metric,
            self.trial_recovery_metric,
            self.trial_asymmetry_metric,
            self.trial_samples_metric,
        ]

        for index, metric in enumerate(metrics):
            layout.addWidget(metric, 3 + index // 4, index % 4)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)
        layout.setColumnStretch(3, 1)

        self.update_assessment_context()
        return panel

    def current_task(self):
        # Return the currently selected standardized assessment task.
        return self.task_select.currentText()

    def current_side(self):
        # Return the selected side for unilateral tasks.
        return self.side_select.currentText()

    def current_threshold(self):
        # Look up the task-specific compensation threshold in degrees.
        return TASK_THRESHOLDS.get(self.current_task(), 8.0)

    def update_assessment_context(self):
        # Refresh threshold and asymmetry text when the selected task changes.
        self.threshold_label.setText(f"{self.current_threshold():.0f} deg")
        self.update_asymmetry_display()

    def reset_trial_display(self):
        # Clear metric tiles before a new assessment recording starts.
        self.trial_rms_metric.set_text("--")
        self.trial_p95_metric.set_text("--")
        self.trial_peak_metric.set_text("--")
        self.trial_outside_metric.set_text("--")
        self.trial_velocity_metric.set_text("--")
        self.trial_recovery_metric.set_text("--")
        self.trial_asymmetry_metric.set_text("--")
        self.trial_samples_metric.set_text("0")

    def start_trial(self):
        # Begin collecting relative trunk-over-pelvis samples for one task trial.
        self.trial_active = True
        self.trial_start_time = time.time()
        self.trial_samples = []
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.task_select.setEnabled(False)
        self.side_select.setEnabled(False)
        self.reset_trial_display()
        self.trial_state_label.setText("recording")

    def stop_trial(self):
        # Stop recording, compute final metrics, and save the trial to CSV.
        if not self.trial_active:
            return

        self.trial_active = False
        metrics = calculate_trial_metrics(self.trial_samples, self.current_threshold())
        task = self.current_task()
        side = self.current_side()

        self.side_results[(task, side)] = metrics
        self.update_trial_display(metrics)
        self.update_asymmetry_display()
        append_trial_csv(task, side, self.current_threshold(), metrics)

        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.task_select.setEnabled(True)
        self.side_select.setEnabled(True)
        self.trial_state_label.setText("saved to CSV")
        self.trial_state_label.setToolTip(str(TRIAL_LOG_PATH))

    def record_trial_sample(self, pose):
        # Add the current pose to the active trial and refresh live metrics.
        elapsed = time.time() - self.trial_start_time
        relative_mag = math.hypot(pose.relative_roll, pose.relative_pitch)

        self.trial_samples.append(
            TrialSample(
                elapsed_s=elapsed,
                relative_roll=pose.relative_roll,
                relative_pitch=pose.relative_pitch,
                relative_mag=relative_mag,
                score=pose.score,
            )
        )

        metrics = calculate_trial_metrics(self.trial_samples, self.current_threshold())
        self.update_trial_display(metrics)
        self.trial_state_label.setText(f"recording {metrics.duration_s:.1f}s")

    def update_trial_display(self, metrics):
        # Show trial metrics using the same units that are written to CSV.
        recovery = "--" if metrics.recovery_s is None else f"{metrics.recovery_s:.1f}s"
        self.trial_rms_metric.set_text(f"{metrics.rms_deg:.1f} deg")
        self.trial_p95_metric.set_text(f"{metrics.p95_deg:.1f} deg")
        self.trial_peak_metric.set_text(f"{metrics.peak_deg:.1f} deg")
        self.trial_outside_metric.set_text(f"{metrics.outside_pct:.0f}%")
        self.trial_velocity_metric.set_text(f"{metrics.velocity_deg_s:.1f} deg/s")
        self.trial_recovery_metric.set_text(recovery)
        self.trial_samples_metric.set_text(str(metrics.sample_count))

    def update_asymmetry_display(self):
        # Compare left and right RMS values after both sides are recorded.
        task = self.current_task()
        left = self.side_results.get((task, "Left"))
        right = self.side_results.get((task, "Right"))

        if left and right:
            asymmetry = abs(left.rms_deg - right.rms_deg)
            self.trial_asymmetry_metric.set_text(f"{asymmetry:.1f} deg")
        else:
            self.trial_asymmetry_metric.set_text("--")

    def refresh(self):
        # Pull the newest pose and push values into each dashboard widget.
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

        if self.trial_active:
            self.record_trial_sample(pose)


def status_color(score):
    # Shared color scale for good, warning, and feedback states.
    if score < 8:
        return "#22c55e"
    if score < 15:
        return "#facc15"
    return "#fb7185"


def set_style(app):
    # Central stylesheet keeps visual polish separate from widget logic.
    app.setStyleSheet(
        """
        QWidget#root {
            background: #0b1020;
            color: #e5e7eb;
        }

        QWidget#assessmentRoot {
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

        QFrame#assessmentPanel {
            background: #111827;
            border: 1px solid #253044;
            border-radius: 8px;
        }

        QFrame#miniMetric {
            background: #0f172a;
            border: 1px solid #263244;
            border-radius: 6px;
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

        QLabel#panelTitle {
            color: #f8fafc;
            font: 800 13px "Segoe UI";
        }

        QLabel#miniTitle {
            color: #94a3b8;
            font: 600 9px "Segoe UI";
        }

        QLabel#miniValue {
            color: #f8fafc;
            font: 800 12px "Segoe UI";
        }

        QLabel#thresholdLabel,
        QLabel#trialState {
            color: #cbd5e1;
            font: 700 10px "Segoe UI";
        }

        QComboBox {
            background: #0f172a;
            color: #e5e7eb;
            border: 1px solid #263244;
            border-radius: 6px;
            padding: 5px 8px;
            font: 600 10px "Segoe UI";
        }

        QPushButton {
            background: #2563eb;
            color: #f8fafc;
            border: 0;
            border-radius: 6px;
            padding: 6px 10px;
            font: 800 10px "Segoe UI";
        }

        QPushButton:disabled {
            background: #334155;
            color: #94a3b8;
        }

        QPushButton#secondaryButton {
            background: #172033;
            color: #cbd5e1;
            border: 1px solid #263244;
        }

        QPushButton#secondaryButton:hover {
            background: #1e293b;
        }
        """
    )


def main():
    # Open the serial port, calibrate, then start the Qt application loop.
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
        # Always release the serial port when the dashboard exits.
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    raise SystemExit(main())
