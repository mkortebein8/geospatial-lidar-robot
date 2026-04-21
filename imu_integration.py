import numpy as np
import pandas as pd


class IMUIntegrator:
    def __init__(self, csv_path="data/imu_output.csv", output_path="data/imu_integration.csv"):
        self.csv_path = csv_path
        self.output_path = output_path

        # Load input CSV
        self.df = pd.read_csv(csv_path)

        # Normalize timestamps to seconds
        ts_max = self.df['timestamp_us'].max()
        if ts_max > 1e12:          # nanoseconds
            self.df['timestamp_us'] /= 1_000_000_000
        elif ts_max > 1e10:        # microseconds
            self.df['timestamp_us'] /= 1_000_000
        elif ts_max > 1e6:         # milliseconds
            self.df['timestamp_us'] /= 1000

        n = len(self.df)
        self.position = np.zeros((n, 2))  # px, py
        self.velocity = np.zeros((n, 2))  # vx, vy

    def quat_to_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    def process(self):
        timestamps = self.df['timestamp_us'].values

        for i in range(1, len(self.df)):
            dt = timestamps[i] - timestamps[i - 1]

            # Reject bad dt values
            if dt <= 0 or dt > 1:
                dt = 0.01  # assume 100 Hz fallback

            # Acceleration
            accel_sensor = np.array([
                self.df.at[i, 'ax'],
                self.df.at[i, 'ay'],
                self.df.at[i, 'az']
            ])

            # Quaternion
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

        # Build output DataFrame
        self.result = self.df.copy()
        self.result["vx"] = self.velocity[:, 0]
        self.result["vy"] = self.velocity[:, 1]
        self.result["px"] = self.position[:, 0]
        self.result["py"] = self.position[:, 1]

        # Save to CSV
        self.result.to_csv(self.output_path, index=False)
        print(f"Saved processed dataset to {self.output_path}")

        return self.result
    
    def location_from_time(self, time): 

        # index of integrated data at the time we're looking for
        exact_index = self.result.index[self.result['timestamp_us']==time].tolist()

        if len(exact_index) > 0:      # we found an exact match for the time

            px = self.result.iloc[exact_index[0]]['px']
            py = self.result.iloc[exact_index[0]]['py']

            return px, py
        
        # look for the closest times and interpolate approximate position
        else:
            #iterate through rows of dataframe
            for i, row in self.result.iterrows():
                # pass if before the time we want
                if row['timestamp_us'] < time:
                    pass
                # once we get to time after the one we're searching for
                else:
                    # get data directly before and after the time we want
                    before_row = self.result.iloc[i-1]
                    after_row = self.result.iloc[i]
                    # math to figure out position in between
                    between_ratio = (time-before_row['timestamp_us'])/(after_row['timestamp_us'] - before_row['timestamp_us'])
                    px = between_ratio * (after_row['px'] - before_row['px']) + before_row['px']
                    py = between_ratio * (after_row['py'] - before_row['py']) + before_row['py']

                    return px, py

''' how to use (second line needed)

imu = IMUIntegrator("imu_data.csv", "imu_output.csv")
result_df = imu.process()
px, py = imu.location_from_time(1234567890)

'''
