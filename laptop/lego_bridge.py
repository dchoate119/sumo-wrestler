"""Relay flip commands from MQTT to the LEGO Education Double Motor (flipper).

The UNO Q publishes one message per flip on dchoate119/sumo/flip. This script
holds the BLE connection to the Double Motor and swings the flipper arm up and
back down. Flips that arrive while one is running are ignored.

All motor calls happen on the main thread; the MQTT thread only sets an event.

Before running: rest the flipper arm in its DOWN position. Startup zeroes the
motor position there, so FLIP_DEG is measured from that point.

    python laptop/lego_bridge.py                     # local broker (localhost)
    python laptop/lego_bridge.py --broker test.mosquitto.org
    python laptop/lego_bridge.py --test              # no MQTT: Enter = flip, q = quit
"""

import argparse
import threading
import time

import legoeducation as le

TOPIC = "dchoate119/sumo/flip"

FLIPPER = le.MOTOR_BOTH  # which side(s) drive the arm: MOTOR_LEFT, MOTOR_RIGHT or MOTOR_BOTH
FLIP_DEG = 120           # how far up the arm swings, degrees from rest (sign sets direction)
FLIP_SPEED = 100         # % speed on the way up: fast for a hard flip
RETURN_SPEED = 50        # % speed back down: gentler so the arm doesn't slam
HOLD_UP_S = 0.15         # pause at the top before returning

card_color = le.LEGO_COLOR_PURPLE
card_serial = '1227'


def connect_motor():
    motor = le.DoubleMotor()
    print("Searching for the Double Motor over Bluetooth...")
    motor.connect(card_color=card_color, card_serial=card_serial)
    if not motor.connected:
        raise SystemExit("Could not connect. Is the motor on and not paired to another app?")
    print(f"Connected (firmware {motor.version_firmware})")
    motor.motor_set_end_state(le.MOTOR_END_STATE_HOLD, motor=FLIPPER)  # arm stays put when stopped
    motor.motor_reset_relative_position(motor=FLIPPER)                 # rest position = 0
    return motor


def flip(motor):
    t0 = time.time()
    motor.motor_run_to_relative_position(FLIP_DEG, motor=FLIPPER, speed=FLIP_SPEED)
    time.sleep(HOLD_UP_S)
    motor.motor_run_to_relative_position(0, motor=FLIPPER, speed=RETURN_SPEED)
    print(f"flip done in {time.time() - t0:.2f} s")


def connect_mqtt(host, port, flip_requested, busy):
    import paho.mqtt.client as mqtt

    def on_connect(client, userdata, flags, reason_code, properties):
        print(f"MQTT: connected to {host}:{port} ({reason_code}), listening on '{TOPIC}'")
        client.subscribe(TOPIC)  # resubscribes after a reconnect too

    def on_message(client, userdata, msg):
        if busy.is_set():
            print("flip ignored: one is already running")
            return
        flip_requested.set()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="sumo-lego-bridge")
    try:  # login for the local broker; the public one needs none
        from mqtt_secrets import PASSWORD, USER
        client.username_pw_set(USER, PASSWORD)
    except ImportError:
        pass
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port)
    client.loop_start()
    return client


def run_test(motor):
    while True:
        cmd = input("Enter = flip, q = quit: ").strip().lower()
        if cmd == "q":
            return
        flip(motor)


def run_bridge(motor, host, port):
    flip_requested = threading.Event()
    busy = threading.Event()
    client = connect_mqtt(host, port, flip_requested, busy)
    try:
        while True:
            if not flip_requested.wait(timeout=0.5):
                if not motor.connected:
                    raise SystemExit("Lost the Bluetooth connection to the motor")
                continue
            busy.set()
            try:
                flip(motor)
            finally:
                flip_requested.clear()  # drop anything that arrived mid-flip
                busy.clear()
    finally:
        client.loop_stop()
        client.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--test", action="store_true", help="flip from the keyboard, no MQTT")
    args = ap.parse_args()

    motor = connect_motor()
    try:
        if args.test:
            run_test(motor)
        else:
            run_bridge(motor, args.broker, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        if motor.connected:
            motor.motor_stop(motor=FLIPPER)
            motor.disconnect()
        print("Disconnected")


if __name__ == "__main__":
    main()