from copter import Copter

copter = Copter()
copter.alt = 100  # take off target altitude
copter.points = [
    (50.443326, 30.448078, 10, 100)  # lat, lon, speed, alt
]
copter.arm()
copter.take_off()
# here, according requirements, drone should change more to ALT_HOLD.
# But, with that mode, it loosing control and falling down, so it keep GUIDED mode
copter.run()
copter.set_yaw(350)
