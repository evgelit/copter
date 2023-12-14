from copter import Copter

DESTINATION_POINT = (50.443326, 30.448078)

CHANNELS_MAPPING = {
    "throttle": {
        "neutral": 1500,
        "gears": {
            1: 125,
            2: 200,
            3: 300,
            4: 400,
            5: 500
        },
    },
    "yaw": {
        "neutral": 1500,
        "gears": {
            1: 22,
            2: 35,
            3: 40,
            4: 75,
            5: 100
        }
    },
    "pitch": {
        "neutral": 1500,
        "gears": {
            1: 100,
            2: 200,
            3: 300,
            4: 400,
            5: 500
            }
        }
}

copter = Copter(channels_mapping=CHANNELS_MAPPING)
copter.arm()
copter.switch_mode(mode="ALT_HOLD")

copter.to_altitude(altitude=100)
copter.to_point(
    lat=DESTINATION_POINT[0],
    long=DESTINATION_POINT[1],
    precision=3
)
copter.rotate_to(yaw=-0.17)
while True:
    copter.run()
