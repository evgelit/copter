from time import sleep
import math
from geopy.distance import geodesic as GD
from dronekit import (
    Vehicle,
    connect,
    VehicleMode
)


class Copter:
    PITCH_CHANNEL = '2'
    THROTTLE_CHANNEL = '3'
    YAW_CHANNEL = '4'

    vehicle: Vehicle

    _pitch: int
    _throttle: int
    _yaw: int

    mapping: dict

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        self._pitch = self.mapping["pitch"]["neutral"] + value

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = self.mapping["throttle"]["neutral"] + value

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        self._yaw = self.mapping["yaw"]["neutral"] + value

    '''
    Init vehicle connection
    '''

    def __init__(
            self,
            channels_mapping: dict,
            ip: str = "tcp:127.0.0.1:5762",
            baud: int = 115200,
            wait_ready: bool = False,
    ):
        self.mapping = channels_mapping
        print(f"Connecting to {ip}")
        self.vehicle = connect(ip=ip, baud=baud, wait_ready=wait_ready)
        print("Vehicle connection established")
        self._pitch = self.mapping["pitch"]["neutral"]
        self._throttle = self.mapping["throttle"]["neutral"]
        self._yaw = self.mapping["yaw"]["neutral"]

    '''
    Arm motors with limited timeout
    '''

    def arm(self, timeout: int = 30) -> None:
        print("Clear all overrides")
        self.vehicle.channels.overrides = {}
        print(" Channel overrides: %s" % self.vehicle.channels.overrides)
        print("Arming")
        timer = 0
        while not self.vehicle.is_armable:
            sleep(1)
            timer += 1
            if timer > timeout:
                raise Exception("Can't arm motors")
        self.vehicle.armed = True
        print("Armed")

    '''
    Change mode
    '''
    def switch_mode(self, mode: str) -> None:
        print(f"Change mode to {mode}")
        self.vehicle.mode = VehicleMode(mode)

    '''
    Keep channels overrides
    '''
    def run(self):
        self.vehicle.channels.overrides = {
            self.PITCH_CHANNEL: self.pitch,
            self.THROTTLE_CHANNEL: self.throttle,
            self.YAW_CHANNEL: self.yaw,
        }

    '''
    Get yaw from 2 points
    '''
    def calculate_yaw(
            self,
            lat1: float,
            long1: float,
            lat2: float,
            long2: float
    ) -> float:
        lat1 = lat1 * math.pi / 180.
        lat2 = lat2 * math.pi / 180.
        long1 = long1 * math.pi / 180.
        long2 = long2 * math.pi / 180.
        cos_lat1 = math.cos(lat1)
        cos_lat2 = math.cos(lat2)
        sin_lat1 = math.sin(lat1)
        sin_lat2 = math.sin(lat2)
        x = (cos_lat1 * sin_lat2) - (sin_lat1 * cos_lat2 * math.cos(long2 - long1))
        y = math.sin(long2 - long1) * cos_lat2
        z = math.degrees(math.atan(-y / x)) + (180 if x < 0 else 0)
        return - math.radians((z + 180.) % 360. - 180.)

    '''
    Go to defined altitude
    '''
    def to_altitude(self, altitude: int) -> None:
        print(f"Reaching target altitude {altitude}")
        while self.vehicle.location.global_relative_frame.alt <= altitude:
            print(f"Altitude {self.vehicle.location.global_relative_frame.alt}")
            alt_delta = altitude - self.vehicle.location.global_relative_frame.alt
            if alt_delta > 10:
                self.throttle = self.mapping["throttle"]["gears"][5]
            elif alt_delta > 5:
                self.throttle = self.mapping["throttle"]["gears"][2]
            else:
                self.throttle = self.mapping["throttle"]["gears"][1]
            self.run()
            sleep(1)
        else:
            print("Target altitude reached")
            self.throttle = 0

    '''
    Get smallest angle between 2 lines
    '''
    def get_result_angle(self, ang1: float, ang2: float) -> float:
        if ang1 / abs(ang1) == ang2 / abs(ang2):
            return max(abs(ang1), abs(ang2)) - min(abs(ang1), abs(ang2))
        else:
            sum_ang = abs(ang1) + abs(ang2)
            if sum_ang > 180:
                return 360 - sum_ang
            else:
                return sum_ang

    '''
    Get shortest rotation direction
    '''
    def get_rotate_direction(self, ang1: float, ang2: float) -> float:
        if max(ang1, ang2) == ang1:
            return -1
        else:
            return 1

    '''
    Rotate to defined yaw
    '''
    def rotate_to(self, yaw) -> None:
        result_angle = self.get_result_angle(math.degrees(self.vehicle.attitude.yaw), math.degrees(yaw))
        while result_angle > 0.9:
            print(f"Adjusting yaw to {result_angle} deg.")
            self.pitch = 0
            direction = self.get_rotate_direction(math.degrees(self.vehicle.attitude.yaw), math.degrees(yaw))
            result_angle = self.get_result_angle(math.degrees(self.vehicle.attitude.yaw), math.degrees(yaw))
            if result_angle > 100:
                self.yaw = self.mapping["yaw"]["gears"][5] * direction
            elif result_angle > 30:
                self.yaw = self.mapping["yaw"]["gears"][4] * direction
            else:
                self.yaw = self.mapping["yaw"]["gears"][1] * direction
            self.run()
            sleep(0.5)
            if result_angle <= 0.9:
                print("Destination yaw reached")
        self.yaw = 0

    '''
    Fly to defined point with specific precision
    '''
    def to_point(self, lat: float, long: float, precision: float = 1) -> None:
        current_point = (
            self.vehicle.location.global_relative_frame.lon,
            self.vehicle.location.global_relative_frame.lat
        )
        destination_point = (long, lat)
        distance = GD(current_point, destination_point).m
        print(f"Flying to specified point")
        while distance > precision:
            current_point = (
                self.vehicle.location.global_relative_frame.lon,
                self.vehicle.location.global_relative_frame.lat
            )
            destination_point = (long, lat)
            distance = GD(current_point, destination_point).m
            print(f"Distance {distance}")
            if distance > 100:
                self.pitch = -self.mapping["pitch"]["gears"][5]
            elif distance > 40:
                self.pitch = -self.mapping["pitch"]["gears"][4]
            elif distance > 30:
                self.pitch = -self.mapping["pitch"]["gears"][3]
            else:
                self.pitch = -self.mapping["pitch"]["gears"][1]
            yaw = self.calculate_yaw(
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon,
                lat,
                long
            )
            self.rotate_to(yaw)
            self.run()
            sleep(1)
        print("Destination point reached")
        self.pitch = 0
