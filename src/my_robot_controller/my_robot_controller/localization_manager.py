import time
import math

class LocalizationManager:
    """
    Decoupled localization manager, GPS geodetic transformer, and diagnostics monitor.
    Standardizes coordinate state extraction and error detection.
    Outputs standard WGS-84 GPS coordinates (Decimal Degrees & DMS).
    """
    EARTH_RADIUS = 6378137.0  # WGS-84 equatorial radius in meters

    def __init__(self, timeout_limit=1.0, drift_yaw_threshold=0.35, 
                 datum_lat=10.775667, datum_lon=106.670889, datum_alt=10.0):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # GPS Reference Datum (Origin for local ENU frame)
        self.datum_lat = datum_lat
        self.datum_lon = datum_lon
        self.datum_alt = datum_alt

        # Current GPS coordinates (WGS-84)
        self.current_lat = datum_lat
        self.current_lon = datum_lon
        self.current_alt = datum_alt
        self.has_direct_gps = False
        self.gps_status = "NO_FIX"
        self.gps_update_time = 0.0

        # IMU fields
        self.imu_yaw = 0.0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_angular_vel_z = 0.0
        self.imu_linear_accel_x = 0.0

        self.last_update_time = time.time()
        self.timeout_limit = timeout_limit
        self.drift_yaw_threshold = drift_yaw_threshold
        
        self.encoder_failed = False
        self.imu_failed = False
        self.status = "OK"

    @classmethod
    def xy_to_latlon(cls, x, y, lat0, lon0):
        """
        Converts local East-North-Up (x, y) coordinates in meters relative to reference (lat0, lon0)
        into WGS-84 Latitude and Longitude in decimal degrees.
        """
        dLat = (y / cls.EARTH_RADIUS) * (180.0 / math.pi)
        dLon = (x / (cls.EARTH_RADIUS * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
        return lat0 + dLat, lon0 + dLon

    @classmethod
    def latlon_to_xy(cls, lat, lon, lat0, lon0):
        """
        Converts WGS-84 Latitude and Longitude in decimal degrees
        into local East-North-Up (x, y) coordinates in meters relative to reference (lat0, lon0).
        """
        dLat = math.radians(lat - lat0)
        dLon = math.radians(lon - lon0)
        y = dLat * cls.EARTH_RADIUS
        x = dLon * cls.EARTH_RADIUS * math.cos(math.radians(lat0))
        return x, y

    @staticmethod
    def format_dms(lat, lon):
        """
        Formats decimal latitude and longitude into standard Degrees Minutes Seconds (DMS) string.
        Example: 10.775667, 106.670889 -> '10°46\'32.40"N, 106°40\'15.20"E'
        """
        def convert_dms(val, pos_dir, neg_dir):
            direction = pos_dir if val >= 0 else neg_dir
            val = abs(val)
            degrees = int(val)
            minutes_full = (val - degrees) * 60.0
            minutes = int(minutes_full)
            seconds = (minutes_full - minutes) * 60.0
            return f"{degrees}°{minutes:02d}'{seconds:05.2f}\"{direction}"

        lat_str = convert_dms(lat, 'N', 'S')
        lon_str = convert_dms(lon, 'E', 'W')
        return f"{lat_str}, {lon_str}"

    def update_odometry(self, msg):
        """
        Parses ROS 2 nav_msgs/Odometry message and computes estimated GPS position from datum origin.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Quaternion to Euler yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        self.last_update_time = time.time()

        # If direct hardware GPS is not available, estimate GPS coordinates from odometry (x, y)
        if not self.has_direct_gps or (time.time() - self.gps_update_time) > 2.0:
            self.current_lat, self.current_lon = self.xy_to_latlon(
                self.current_x, self.current_y, self.datum_lat, self.datum_lon
            )
            self.current_alt = self.datum_alt + msg.pose.pose.position.z
            self.gps_status = "ODOM_ESTIMATED"
        
        # Simple diagnostic range check
        if abs(self.linear_velocity) > 5.0 or abs(self.angular_velocity) > 6.0:
            self.encoder_failed = True
        else:
            self.encoder_failed = False

    def update_gps(self, msg):
        """
        Parses ROS 2 sensor_msgs/msg/NavSatFix message.
        """
        # Check for NaN / invalid values
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        # If sensor or bridge returns (0, 0), fallback to geodetic calculation from local (x, y) pose
        if abs(msg.latitude) < 1e-5 and abs(msg.longitude) < 1e-5:
            self.current_lat, self.current_lon = self.xy_to_latlon(
                self.current_x, self.current_y, self.datum_lat, self.datum_lon
            )
            self.current_alt = self.datum_alt
            self.has_direct_gps = True
            self.gps_status = "FIX"
            self.gps_update_time = time.time()
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude if not math.isnan(msg.altitude) else self.datum_alt
        
        self.has_direct_gps = True
        self.gps_update_time = time.time()

        # Check fix status (NavSatStatus: STATUS_NO_FIX = -1, STATUS_FIX = 0, STATUS_SBAS_FIX = 1, STATUS_GBAS_FIX = 2)
        status_val = msg.status.status if hasattr(msg, 'status') else 0
        if status_val >= 0:
            self.gps_status = "FIX"
        else:
            self.gps_status = "NO_FIX"

    def update_imu(self, msg):
        """
        Parses ROS 2 sensor_msgs/Imu message into 3D Euler angles (Roll, Pitch, Yaw)
        for terrain slope (uphill/downhill) and side tilt monitoring.
        """
        q = msg.orientation
        
        # Roll (x-axis rotation: side tilt)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation: uphill/downhill slope angle)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.imu_pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.imu_pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation: heading compass)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.imu_angular_vel_z = msg.angular_velocity.z
        self.imu_linear_accel_x = msg.linear_acceleration.x
        
        yaw_diff = abs(math.atan2(math.sin(self.imu_yaw - self.current_yaw), math.cos(self.imu_yaw - self.current_yaw)))
        if yaw_diff > self.drift_yaw_threshold:
            self.status = "DRIFT_DETECTED"
            
        self.last_update_time = time.time()

    def get_gps_coordinates(self):
        """
        Returns standardized WGS-84 GPS coordinate dictionary.
        """
        dms_str = self.format_dms(self.current_lat, self.current_lon)
        return {
            "latitude": round(self.current_lat, 8),
            "longitude": round(self.current_lon, 8),
            "altitude": round(self.current_alt, 3),
            "dms": dms_str,
            "status": self.gps_status,
            "source": "DIRECT_SENSOR" if self.has_direct_gps and (time.time() - self.gps_update_time) <= 2.0 else "ODOM_ESTIMATED",
            "datum_origin": {
                "latitude": self.datum_lat,
                "longitude": self.datum_lon,
                "altitude": self.datum_alt
            },
            "timestamp": self.last_update_time
        }

    def get_pose(self):
        """
        Returns standardized Localization output dictionary including 3D IMU Euler angles and GPS coordinates.
        """
        self.check_diagnostics()
        gps_info = self.get_gps_coordinates()
        
        return {
            "x": self.current_x,
            "y": self.current_y,
            "yaw": self.current_yaw,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "imu": {
                "roll": self.imu_roll,
                "pitch": self.imu_pitch,
                "yaw": self.imu_yaw,
                "roll_deg": round(math.degrees(self.imu_roll), 2),
                "pitch_deg": round(math.degrees(self.imu_pitch), 2),
                "yaw_deg": round(math.degrees(self.imu_yaw), 2),
                "angular_vel_z": self.imu_angular_vel_z,
                "linear_accel_x": self.imu_linear_accel_x
            },
            "gps": gps_info,
            "status": self.status,
            "timestamp": self.last_update_time
        }

    def check_diagnostics(self):
        """
        Monitors for update timeouts, drift, or sensor failure.
        """
        dt = time.time() - self.last_update_time
        if dt > self.timeout_limit:
            self.status = "TIMEOUT"
        elif self.encoder_failed or self.imu_failed:
            self.status = "SENSOR_FAILED"
        elif self.status != "DRIFT_DETECTED":
            self.status = "OK"

    def reset(self):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.current_lat = self.datum_lat
        self.current_lon = self.datum_lon
        self.current_alt = self.datum_alt
        self.has_direct_gps = False
        self.gps_status = "NO_FIX"
        self.gps_update_time = 0.0
        self.last_update_time = time.time()
        self.encoder_failed = False
        self.imu_failed = False
        self.status = "OK"
