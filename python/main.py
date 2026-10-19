"""UNO Q side: hardware test.

Spins each motor forward on its own, then both, so wiring and direction can
be checked (fix with INVERT_M1/M2 in the sketch). After that it just logs the
ultrasonic and accelerometer readings. Lift the wheels off the ground first.
"""

import time

from arduino.app_utils import App, Bridge, Logger

TEST_SPEED = 100     # of 255
TEST_S = 1.5         # per step
READ_HZ = 5

# (label, left, right)
STEPS = [("left motor forward", TEST_SPEED, 0),
         ("right motor forward", 0, TEST_SPEED),
         ("both forward", TEST_SPEED, TEST_SPEED)]

logger = Logger("sumo-wrestler")


def motor_test():
    for label, left, right in STEPS:
        logger.info(f"motor test: {label}")
        end = time.time() + TEST_S
        while time.time() < end:
            Bridge.call("drive", left, right)  # keep feeding the 200 ms watchdog
            time.sleep(0.05)
        Bridge.call("drive", 0, 0)
        time.sleep(0.5)
    logger.info("motor test done")


tested = False


def loop():
    global tested
    if not tested:
        logger.info(f"imu: {Bridge.call('imu_check')}")
        motor_test()
        tested = True

    cm = Bridge.call("distance")
    imu = Bridge.call("imu_read")
    dist = "no echo" if cm < 0 else f"{cm:6.1f} cm"
    logger.info(f"distance: {dist} | imu (g, deg/s): {imu or 'not found'}")
    time.sleep(1 / READ_HZ)


try:
    App.run(user_loop=loop)
finally:
    Bridge.call("drive", 0, 0)
