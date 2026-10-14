"""UNO Q side: ultrasonic sensor test.

Asks the sketch for a distance reading a few times a second and logs it.
"""

import time

from arduino.app_utils import App, Bridge, Logger

READ_HZ = 5

logger = Logger("sumo-wrestler")


def loop():
    cm = Bridge.call("distance")
    if cm < 0:
        logger.info("distance: no echo")
    else:
        logger.info(f"distance: {cm:6.1f} cm")
    time.sleep(1 / READ_HZ)


App.run(user_loop=loop)
