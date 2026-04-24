import csv
import time
import serial
import threading


class IMUSerialLogger:
    def __init__(self, port, baud=115200, csv_path="imu_data.csv"):
        self.port = port
        self.baud = baud
        self.csv_path = csv_path

        self.ser = None
        self.thread = None
        self.running = False

        # Prepare CSV with header
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp_us",
                "ax", "ay", "az",
                "qw", "qx", "qy", "qz"
            ])

    def _open_serial(self):
        if self.ser is None:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)

    def _close_serial(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def _worker(self):
        """Background thread that reads and logs IMU data."""
        self._open_serial()
        print("IMU logging started.")

        while self.running:
            line = self.ser.readline().decode("utf-8").strip()

            if not line:
                continue

            parts = line.split(",")

            if len(parts) != 7:
                print("Skipping malformed line:", line)
                continue

            try:
                ax, ay, az, qw, qx, qy, qz = map(float, parts)
            except ValueError:
                print("Skipping non-numeric line:", line)
                continue

            timestamp_us = time.time()

            with open(self.csv_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp_us,
                    ax, ay, az,
                    qw, qx, qy, qz
                ])

        print("IMU logging stopped.")
        self._close_serial()

    def start(self):
        """Start logging in a background thread."""
        if self.running:
            print("Logger already running.")
            return

        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop logging and wait for thread to finish."""
        if not self.running:
            print("Logger is not running.")
            return

        self.running = False
        self.thread.join()
        self.thread = None



''' example usage:
from imu_logger_threaded import IMUSerialLogger

logger = IMUSerialLogger(port='/dev/ttyACM0', baud=115200, csv_path="imu_data.csv")

logger.start()
time.sleep(10)   # collect data for 10 seconds, only needed for example
logger.stop()
'''
