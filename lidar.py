from rplidar import RPLidar
import time
import threading
import numpy as np

class LidarRecording:
    # Initialize lidar class with port
    def __init__(self, port='/dev/ttyUSB0'): # Windows port r'\\.\COM3'
        # Save attribute
        self.start_time = None
        self.end_time = None
        self.port = port
        self.thread = None
        self.recording = False
        self.used = False       # True when this instance has recorded some data

        # Establish data structures
        self.angle_data = []        # degrees
        self.dist_data = []         # meters
        self.time_data = np.array([]) # seconds
        
        # establish lidar collection
        self.lidar = RPLidar(self.port)
    
    # Internal method to run data collection on its own thread
    def _collect_data(self):
        self.lidar.connect()
        self.lidar.start_motor()
        self.start_time = time.time()
        
        try:
            for scan in self.lidar.iter_scans(max_buf_meas = 1000):
                if not self.recording:
                    break
                for quality, angle, distance in scan:
                    if quality > 0:     # when quality = 0, this means that the data is bad and should be ignored
                        self.dist_data.append(distance / 1000) # Adjust to meters
                        self.angle_data.append(angle)

        except Exception as e:
            print(f"Lidar error: {e}")

        finally:
            self.end_time = time.time()
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()

    # Start data recording
    def start(self):
        if self.used:
            print('This instance has already recorded data. Please create a new instance to record new data.')
        elif not self.recording:
            self.recording = True
            self.thread = threading.Thread(target=self._collect_data)
            self.thread.start()

    # End data recording and calculate time stamps
    def stop(self):
        if self.recording:
            self.recording = False
            if self.thread:
                self.thread.join()

            self.dist_data = np.array(self.dist_data)
            self.angle_data = np.array(self.angle_data)

            # Interpolate times
            self.time_data = np.linspace(0, (self.end_time - self.start_time), num=len(self.dist_data))

        self.used = True
