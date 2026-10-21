"""Keyboard teleop: drive the robot from a small window, over MQTT.

While a control key is held, publishes {"l", "r", "flip"} to TOPIC at SEND_HZ.
Nothing goes out when no key is down, so the board drops back to auto 0.5 s
after you let go. Space is the hold button: it sends zeros, so the robot stands
still under teleop instead of returning to auto. Teleop only runs while
track.py is in auto (idle still wins), and the board cancels forward motion
toward the ring edge.

    W / S      forward / back
    A / D      turn left / right (combine with W/S to arc)
    Shift      full speed
    Space      hold (stand still under teleop)
    F          flip (the board applies the flip cooldown)
    Esc        quit

    python laptop/teleop.py                    # local broker (localhost)
    python laptop/teleop.py --broker test.mosquitto.org
    python laptop/teleop.py --test             # no MQTT: prints what it would send
"""

import argparse
import json

import pygame

TOPIC = "dchoate119/sumo/teleop"
SEND_HZ = 20
DRIVE_SPEED = 150  # of 255
TURN_SPEED = 100   # added to one wheel, taken from the other
BOOST_SPEED = 255  # drive speed with Shift


def command(keys):
    """(l, r, flip) from the held keys, or None if no control key is down."""
    fwd = keys[pygame.K_w] - keys[pygame.K_s]
    turn = keys[pygame.K_d] - keys[pygame.K_a]
    hold = keys[pygame.K_SPACE]
    flip = bool(keys[pygame.K_f])
    if not (fwd or turn or hold or flip):
        return None
    if hold:
        return 0, 0, flip
    boost = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
    speed = (BOOST_SPEED if boost else DRIVE_SPEED) * fwd
    l = max(-255, min(255, speed + TURN_SPEED * turn))
    r = max(-255, min(255, speed - TURN_SPEED * turn))
    return l, r, flip


def connect_mqtt(host, port):
    import paho.mqtt.client as mqtt

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="sumo-teleop")
    try:  # login for the local broker; the public one needs none
        from mqtt_secrets import PASSWORD, USER
        client.username_pw_set(USER, PASSWORD)
    except ImportError:
        pass
    client.connect(host, port)
    client.loop_start()
    print(f"MQTT: publishing to {host}:{port} topic '{TOPIC}'")
    return client


def draw(screen, font, cmd):
    screen.fill((25, 30, 40))
    lines = ["WASD drive   Shift fast   Space hold   F flip   Esc quit", ""]
    if cmd is None:
        lines.append("not sending (auto)")
    else:
        l, r, flip = cmd
        lines.append(f"l {l:4d}   r {r:4d}" + ("   FLIP" if flip else ""))
    for i, text in enumerate(lines):
        color = (255, 255, 255) if cmd is not None or i < 2 else (150, 150, 150)
        screen.blit(font.render(text, True, color), (16, 16 + 28 * i))
    pygame.display.flip()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--test", action="store_true", help="no MQTT, print messages instead")
    args = ap.parse_args()

    client = None if args.test else connect_mqtt(args.broker, args.port)
    pygame.init()
    screen = pygame.display.set_mode((560, 110))
    pygame.display.set_caption("sumo teleop (keep this window focused)")
    font = pygame.font.SysFont("monospace", 18)
    clock = pygame.time.Clock()
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    return
            cmd = command(pygame.key.get_pressed())
            if cmd is not None:
                l, r, flip = cmd
                msg = json.dumps({"l": l, "r": r, "flip": flip})
                if client:
                    client.publish(TOPIC, msg, qos=0)
                else:
                    print(msg)
            draw(screen, font, cmd)
            clock.tick(SEND_HZ)
    finally:
        pygame.quit()
        if client:
            client.loop_stop()
            client.disconnect()


if __name__ == "__main__":
    main()
