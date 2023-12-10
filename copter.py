from time import sleep
from dronekit import (
    mavutil,
    Vehicle,
    connect,
    LocationGlobalRelative,
    VehicleMode
)


class Copter:

    vehicle: Vehicle
    alt: float
    points: list

    '''
    Init vehicle connection
    '''
    def __init__(
            self,
            ip: str = "tcp:127.0.0.1:5762",
            baud: int = 115200,
            wait_ready: bool = False
    ):
        print(f"Connecting to {ip}")
        self.vehicle = connect(ip=ip, baud=baud, wait_ready=wait_ready)
        print("Vehicle connection established")

    '''
    Arm motors with limited timeout
    '''
    def arm(self, timeout: int = 30) -> None:
        print("Arming")
        timer = 0
        while not self.vehicle.is_armable:
            sleep(1)
            timer += 1
            if timer > timeout:
                raise Exception("Can't arm motors")
        self.switch_mode("GUIDED")
        self.vehicle.armed = True
        print("Armed")

    '''
    Take off and reach target altitude
    '''
    def take_off(self, wait_altitude_reached: bool = True) -> None:
        print("Taking off")
        self.vehicle.simple_takeoff(self.alt)
        if wait_altitude_reached is False:
            return
        while self.vehicle.location.global_relative_frame.alt < self.alt * 0.99:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            sleep(1)
        print("Target altitude reached")

    def switch_mode(self, mode: str) -> None:
        print(f"Change mode to {mode}")
        self.vehicle.mode = VehicleMode(mode)

    def run(self):
        for point in self.points:
            location = LocationGlobalRelative(point[0], point[1], alt=point[3])
            print(f"Follow to point: {point[0]}, {point[1]}")
            self.vehicle.simple_goto(location, airspeed=point[2])
            while (round(self.vehicle.location.global_relative_frame.lat, 5) != round(point[0], 5)
                   and round(self.vehicle.location.global_relative_frame.lon, 5) != round(point[1], 5)):
                print(self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon)
                sleep(1)
            print("Target point reached")

    def set_yaw(self, yaw):
        print(f"Yaw: {yaw}")
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,
            yaw,
            0,
            1,
            0,
            0, 0, 0)
        self.vehicle.send_mavlink(msg)