#### written by copilot

import numpy as np
import pandas as pd


class IMUIntegrator:
    def __init__(self, csv_path):
        self.df = pd.read_csv(csv_path)

        # Ensure timestamps are in seconds
        if self.df['timestamp'].max() > 1e6:
            self.df['timestamp'] = self.df['timestamp'] / 1_000_000

        # Output arrays
        self.position = np.zeros((len(self.df), 2))  # px, py
        self.velocity = np.zeros((len(self.df), 2))  # vx, vy

    def quat_to_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    def process(self):
        timestamps = self.df['timestamp'].values

        for i in range(1, len(self.df)):
            dt = timestamps[i] - timestamps[i - 1]

            # Read acceleration
            accel_sensor = np.array([
                self.df.at[i, 'ax'],
                self.df.at[i, 'ay'],
                self.df.at[i, 'az']
            ])

            # Read quaternion
            q = np.array([
                self.df.at[i, 'qw'],
                self.df.at[i, 'qx'],
                self.df.at[i, 'qy'],
                self.df.at[i, 'qz']
            ])

            # Rotate acceleration into world frame
            R = self.quat_to_matrix(q)
            accel_world = R @ accel_sensor

            ax_w, ay_w = accel_world[0], accel_world[1]

            # Integrate velocity
            self.velocity[i, 0] = self.velocity[i-1, 0] + ax_w * dt
            self.velocity[i, 1] = self.velocity[i-1, 1] + ay_w * dt

            # Integrate position
            self.position[i, 0] = self.position[i-1, 0] + self.velocity[i, 0] * dt
            self.position[i, 1] = self.position[i-1, 1] + self.velocity[i, 1] * dt

        return self.position, self.velocity
