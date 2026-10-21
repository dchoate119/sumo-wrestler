"""UNO Q side: behavior tree driven by the laptop's state messages.

Subscribes to the state topic from laptop/track.py, ticks the tree in bt.py at
TICK_HZ, and sends the resulting drive command to the sketch. Space in track.py
switches between idle and auto. The running behavior goes back out on
BEHAVIOR_TOPIC so track.py can show it, and each flip goes out on FLIP_TOPIC for
laptop/lego_bridge.py. The ultrasonic is read every tick. The broker address and login come from the
gitignored python/mqtt_secrets.py (BROKER, USER, PASSWORD).
"""

import json
import threading
import time

import paho.mqtt.client as mqtt
from arduino.app_utils import App, Bridge, Logger

from bt import build_tree
from mqtt_secrets import BROKER, PASSWORD, USER

PORT = 1883
TOPIC = "dchoate119/sumo/state"
TELEOP_TOPIC = "dchoate119/sumo/teleop"      # {"l", "r", "flip"} from laptop/teleop.py, only while keys are held
BEHAVIOR_TOPIC = "dchoate119/sumo/behavior"  # running behavior, sent on change for track.py
FLIP_TOPIC = "dchoate119/sumo/flip"          # one message per flip, for laptop/lego_bridge.py
TICK_HZ = 20        # also feeds the sketch's 200 ms motor watchdog
STALE_S = 0.3       # state older than this (by arrival time) is not fresh
TELEOP_S = 0.5      # teleop is active while its last message is younger than this
US_LOG_S = 0.5      # log the ultrasonic this often, to tune FLIP_CM

logger = Logger("sumo-wrestler")

latest = {}         # last decoded state message
latest_rx = 0.0     # time.monotonic() when it arrived
teleop = {}         # last decoded teleop message
teleop_rx = -1e9    # time.monotonic() when it arrived
lock = threading.Lock()


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        logger.warning(f"MQTT connect failed: {reason_code}")
        return
    logger.info(f"MQTT connected to {BROKER}, subscribing to {TOPIC} and {TELEOP_TOPIC}")
    client.subscribe([(TOPIC, 0), (TELEOP_TOPIC, 0)])  # (re)subscribe on every reconnect


def on_message(client, userdata, msg):
    global latest, latest_rx, teleop, teleop_rx
    try:
        data = json.loads(msg.payload)
    except ValueError:
        logger.warning(f"Bad payload: {msg.payload[:80]!r}")
        return
    with lock:
        if msg.topic == TELEOP_TOPIC:
            teleop, teleop_rx = data, time.monotonic()
        else:
            latest, latest_rx = data, time.monotonic()


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set(USER, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message
client.connect_async(BROKER, PORT)  # retries in the background if the network isn't up yet
client.loop_start()

tree = build_tree()
last_behavior = None
last_behavior_pub = 0.0
last_flip = -1e9    # time.monotonic() of the last flip sent
last_us_log = 0.0


def loop():
    global last_behavior, last_behavior_pub, last_flip, last_us_log
    now = time.monotonic()
    with lock:
        bb = dict(latest)
        bb["fresh"] = now - latest_rx < STALE_S
        if now - teleop_rx < TELEOP_S:
            bb["teleop"] = dict(teleop)
    bb["now"] = now
    bb["last_flip"] = last_flip
    bb["ultrasonic_cm"] = Bridge.call("distance")
    tree.tick(bb)

    kind, *args = bb["cmd"]
    if kind == "drive":
        Bridge.call("drive", *(int(a) for a in args))
    if now - last_us_log >= US_LOG_S:
        logger.info(f"ultrasonic: {bb['ultrasonic_cm']:.1f} cm ({bb['behavior']})")
        last_us_log = now
    if bb.get("flip"):
        client.publish(FLIP_TOPIC, "flip", qos=0)
        last_flip = now
        logger.info(f"flip at {bb['ultrasonic_cm']:.1f} cm")
    if bb["behavior"] != last_behavior:
        logger.info(f"behavior: {bb['behavior']}")
    # on change, and every second in case an earlier one went out before MQTT connected
    if bb["behavior"] != last_behavior or now - last_behavior_pub > 1.0:
        client.publish(BEHAVIOR_TOPIC, bb["behavior"], qos=0, retain=True)
        last_behavior, last_behavior_pub = bb["behavior"], now
    time.sleep(1 / TICK_HZ)


try:
    App.run(user_loop=loop)
finally:
    Bridge.call("drive", 0, 0)
    client.publish(BEHAVIOR_TOPIC, "App stopped", qos=0, retain=True).wait_for_publish(1)
    client.loop_stop()
