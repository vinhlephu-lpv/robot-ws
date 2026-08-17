#!/usr/bin/env python3
"""
Telemetry and Event Logger Utility for ROS 2 LuanVan Robot.
Exports telemetry metrics (FSM, Pose, Steering Angle, Velocities, GPS) to structured CSV and formatted text log files.
"""

import os
import time
import csv
from datetime import datetime


class TelemetryLogger:
    def __init__(self, log_dir=None, enable_csv=True, enable_text_log=True):
        if log_dir is None:
            # Default to LuanVan/logs directory
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            log_dir = os.path.join(base_dir, 'logs')

        self.log_dir = log_dir
        self.enable_csv = enable_csv
        self.enable_text_log = enable_text_log
        
        os.makedirs(self.log_dir, exist_ok=True)
        
        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(self.log_dir, f"telemetry_{timestamp_str}.csv")
        self.log_filename = os.path.join(self.log_dir, f"system_run_{timestamp_str}.log")
        
        self.csv_headers = [
            'Time',
            'Unix_Timestamp',
            'FSM_State',
            'Pos_X_m',
            'Pos_Y_m',
            'Yaw_rad',
            'Steer_Angle_deg',
            'Linear_Vel_mps',
            'Angular_Vel_radps',
            'IMU_Yaw_rad',
            'IMU_Angular_Vel_z',
            'IMU_Accel_x',
            'Confidence',
            'Dist_Traveled_m',
            'GPS_Latitude',
            'GPS_Longitude',
            'GPS_Altitude_m',
            'GPS_DMS',
            'GPS_Status',
            'Event'
        ]

        if self.enable_csv and not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_headers)

        if self.enable_text_log:
            start_header = (
                "=" * 80 + "\n"
                f" ROBOT SYSTEM RUN LOG | STARTED: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                f" CSV Telemetry File : {self.csv_filename}\n"
                f" Text System Log File: {self.log_filename}\n"
                + "=" * 80 + "\n"
            )
            try:
                with open(self.log_filename, mode='w') as f:
                    f.write(start_header)
            except Exception as e:
                print(f"[TelemetryLogger Error] Failed to write header: {e}")

            self.log_event("SYSTEM_START", "Telemetry and Logging system started cleanly.")

    def log_telemetry(self, data: dict):
        """
        Appends a clean, formatted telemetry record to the CSV log file.
        """
        if not self.enable_csv:
            return

        now_sec = time.time()
        time_str = datetime.fromtimestamp(now_sec).strftime("%H:%M:%S.%f")[:-3]

        row = [
            time_str,
            f"{now_sec:.3f}",
            data.get('fsm_state', 'UNKNOWN'),
            f"{data.get('x', 0.0):.3f}",
            f"{data.get('y', 0.0):.3f}",
            f"{data.get('yaw', 0.0):.3f}",
            f"{data.get('steering_angle_deg', 0.0):.2f}",
            f"{data.get('linear_velocity', 0.0):.3f}",
            f"{data.get('angular_velocity', 0.0):.3f}",
            f"{data.get('imu_yaw', 0.0):.3f}",
            f"{data.get('imu_angular_vel_z', 0.0):.3f}",
            f"{data.get('imu_accel_x', 0.0):.3f}",
            f"{data.get('confidence', 0.0):.3f}",
            f"{data.get('distance_traveled', 0.0):.2f}",
            f"{data.get('gps_latitude', 0.0):.8f}",
            f"{data.get('gps_longitude', 0.0):.8f}",
            f"{data.get('gps_altitude', 0.0):.2f}",
            data.get('gps_dms', ''),
            data.get('gps_status', 'NO_FIX'),
            data.get('event', '')
        ]

        try:
            with open(self.csv_filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
        except Exception as e:
            print(f"[TelemetryLogger Error] Failed to write CSV: {e}")

    def log_event(self, event_type: str, message: str):
        """
        Logs a key system event to the text log file with clear time separation.
        """
        if not self.enable_text_log:
            return

        time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_line = f"[{time_str}] [{event_type:^15s}] {message}\n"
        
        try:
            with open(self.log_filename, mode='a') as f:
                f.write(log_line)
        except Exception as e:
            print(f"[TelemetryLogger Error] Failed to write text log: {e}")
