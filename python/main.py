"""UNO Q side: behavior tree driven by the laptop's state messages.

Subscribes to the state topic from laptop/track.py, ticks the tree in bt.py at
TICK_HZ, and sends the resulting drive command to the sketch. Space in track.py
switches between idle and auto. The broker address and login come from the
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
TICK_HZ = 20        # also feeds the sketch's 200 ms motor watchdog
STALE_S = 0.3       # state older than this (by arrival time) is not fresh

logger = Logger("sumo-wrestler")

latest = {}         # last decoded state message
latest_rx = 0.0     # time.monotonic() when it arrived
lock = threading.Lock()


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        logger.warning(f"MQTT connect failed: {reason_code}")
        return
    logger.info(f"MQTT connected to {BROKER}, subscribing to {TOPIC}")
    client.subscribe(TOPIC, qos=0)  # (re)subscribe on every reconnect


def on_message(client, userdata, msg):
    global latest, latest_rx
    try:
        data = json.loads(msg.payload)
    except ValueError:
        logger.warning(f"Bad payload: {msg.payload[:80]!r}")
        return
    with lock:
        latest = data
        latest_rx = time.monotonic()


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set(USER, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message
client.connect_async(BROKER, PORT)  # retries in the background if the network isn't up yet
client.loop_start()

tree = build_tree()
last_behavior = None


def loop():
    global last_behavior
    with lock:
        bb = dict(latest)
        bb["fresh"] = time.monotonic() - latest_rx < STALE_S
    tree.tick(bb)

    kind, *args = bb["cmd"]
    if kind == "drive":
        Bridge.call("drive", *(int(a) for a in args))
    if bb["behavior"] != last_behavior:
        logger.info(f"behavior: {bb['behavior']}")
        last_behavior = bb["behavior"]
    time.sleep(1 / TICK_HZ)


try:
    App.run(user_loop=loop)
finally:
    Bridge.call("drive", 0, 0)
    client.loop_stop()
