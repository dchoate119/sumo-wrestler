"""Live tracker: our robot's pose and the opponent's position, in floor cm.

Needs calibration.json from calibrate.py. Every frame, the floor tags in view
give a fresh image -> floor homography (the last good one is reused if none are
seen), and the robot tag is mapped through it. The opponent is found by
opponent.py. With --mqtt, every frame's state goes out as one JSON message.

    t              laptop time.time() when the frame was processed
    mode           "idle" (robot holds still) or "auto" (behavior tree runs); space toggles
    seen          our robot tag was found; if false, the fields up to edge are missing
    held          seen, but the tag was lost < POSE_HOLD_S ago: the pose is its last one
    x, y           robot tag center in floor cm (see geometry.py for the frame)
    heading        direction the robot faces (tag's top edge + robot_heading_offset_deg),
                   degrees from +x toward +y
    edge           distance from the center to the ring's inner edge, cm (negative = past it)
    edge_bearing   direction to the nearest point of the inner edge   } degrees from our heading,
    center_bearing direction to the ring's center                     } [-180, 180), 0 = ahead
    out            all 4 robot tag corners past the inner edge for OUT_HOLD_S; stays true
    opp_seen       an opponent blob was found; if false, the opp_ fields are missing
    opp_x, opp_y   opponent blob center in floor cm
    opp_edge       opponent's distance to the inner edge, cm
    opp_dist       cm from us to the opponent                         } only when seen too
    opp_bearing    direction to the opponent, degrees from our heading }

The robot tag sits above the floor, so its position is skewed slightly away
from the point under the camera. Fine for edge distances of a few cm.

    python laptop/track.py
    python laptop/track.py --camera 0
    python laptop/track.py --mqtt                     # publish to the public test broker
    python laptop/track.py --mqtt --broker localhost  # once a local broker runs

Keys in the preview window: space = start/stop (auto/idle), o = show opponent mask,
r = reset out, q (or Esc) = quit. Starts in idle.
"""

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from geometry import bearing, detect, make_detector, nearest_on_polygon, to_floor
from opponent import OpponentFinder

HERE = Path(__file__).parent

# Public broker for now; a local Mosquitto (--broker localhost) has steadier latency.
# Topic is namespaced because anyone can publish on a public broker.
DEFAULT_BROKER = "test.mosquitto.org"
TOPIC = "dchoate119/sumo/state"
BEHAVIOR_TOPIC = "dchoate119/sumo/behavior"  # the board's running behavior, shown on screen
OUT_HOLD_S = 0.5  # robot must be out this long before "out" goes true; then it latches
POSE_HOLD_S = 0.5  # tag lost: keep reporting the last pose this long (held = true)

GREEN = (0, 255, 0)
ORANGE = (0, 165, 255)
MAGENTA = (255, 0, 255)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
GRAY = (150, 150, 150)


def load_calibration(path):
    """Returns ({tag id: 4x2 corners in floor cm}, ring polygon as Nx1x2 float32)."""
    cal = json.loads(path.read_text())
    floor = {int(i): np.array(t["corners_cm"]) for i, t in cal["floor_tags"].items()}
    ring = np.array(cal["ring_cm"], dtype=np.float32).reshape(-1, 1, 2)
    return floor, ring


def homography(tags, floor):
    """Image -> floor homography from every visible floor tag, or None if none are visible."""
    seen = [i for i in tags if i in floor]
    if not seen:
        return None
    src = np.vstack([tags[i] for i in seen])
    dst = np.vstack([floor[i] for i in seen])
    H, _ = cv2.findHomography(src, dst)  # least squares over all corners
    return H


def robot_pose(corners_cm, ring, ring_center, heading_offset):
    """Pose dict from the robot tag's corners in floor cm. "out" here is this frame only."""
    x, y = corners_cm.mean(axis=0)
    front = corners_cm[:2].mean(axis=0) - corners_cm[2:].mean(axis=0)  # bottom edge -> top edge
    heading = math.degrees(math.atan2(front[1], front[0])) + heading_offset
    heading = (heading + 180) % 360 - 180
    edge = cv2.pointPolygonTest(ring, (float(x), float(y)), True)  # + inside, - outside
    ex, ey = nearest_on_polygon(ring, (x, y))
    out = all(cv2.pointPolygonTest(ring, (float(cx), float(cy)), False) < 0
              for cx, cy in corners_cm)
    return {
        "x": round(float(x), 1),
        "y": round(float(y), 1),
        "heading": round(heading, 1),
        "edge": round(float(edge), 1),
        "edge_bearing": round(bearing(x, y, heading, ex, ey), 1),
        "center_bearing": round(bearing(x, y, heading, *ring_center), 1),
        "out": out,
    }


def opponent_state(opp, pose, ring):
    """Opponent fields of the state, relative ones only when our pose is known too."""
    if opp is None:
        return {"opp_seen": False}
    x, y = opp
    state = {
        "opp_seen": True,
        "opp_x": round(x, 1),
        "opp_y": round(y, 1),
        "opp_edge": round(float(cv2.pointPolygonTest(ring, (x, y), True)), 1),
    }
    if pose is not None:
        state["opp_dist"] = round(math.hypot(x - pose["x"], y - pose["y"]), 1)
        state["opp_bearing"] = round(bearing(pose["x"], pose["y"], pose["heading"], x, y), 1)
    return state


def connect_mqtt(host, port, board):
    """Connect and start publishing; the board's behavior lands in board["behavior"]."""
    import paho.mqtt.client as mqtt  # only needed with --mqtt

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="sumo-laptop")
    try:  # login for the local broker; the public one needs none
        from mqtt_secrets import PASSWORD, USER
        client.username_pw_set(USER, PASSWORD)
    except ImportError:
        pass
    client.on_connect = lambda c, *_: c.subscribe(BEHAVIOR_TOPIC, qos=0)
    client.on_message = lambda c, u, msg: board.update(behavior=msg.payload.decode(errors="replace"))
    client.connect(host, port)
    client.loop_start()
    print(f"MQTT: publishing to {host}:{port} topic '{TOPIC}'")
    return client


def draw(frame, tags, H, ring, robot_id, state, fps, behavior):
    if H is not None:
        ring_px = to_floor(np.linalg.inv(H), ring.reshape(-1, 2))
        cv2.polylines(frame, [ring_px.astype(int)], True, MAGENTA, 2)
    for tag_id, c in tags.items():
        color = ORANGE if tag_id == robot_id else GREEN
        cv2.polylines(frame, [c.astype(int)], True, color, 2)
    s = state
    if s["seen"] and not s["held"]:  # heading arrow, 15 cm long on the floor
        h = math.radians(s["heading"])
        ends = to_floor(np.linalg.inv(H), [[s["x"], s["y"]],
                                           [s["x"] + 15 * math.cos(h), s["y"] + 15 * math.sin(h)]])
        cv2.arrowedLine(frame, tuple(ends[0].astype(int)), tuple(ends[1].astype(int)),
                        ORANGE, 3, tipLength=0.3)
    if s["opp_seen"]:
        p = tuple(to_floor(np.linalg.inv(H), [[s["opp_x"], s["opp_y"]]])[0].astype(int))
        cv2.drawMarker(frame, p, RED, cv2.MARKER_CROSS, 30, 3)
        label = f"opp  edge {s['opp_edge']:.0f}"
        if "opp_dist" in s:
            label += f"  {s['opp_dist']:.0f} cm @ {s['opp_bearing']:.0f}"
        cv2.putText(frame, label, (p[0] + 15, p[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)

    if H is None:
        text = "no floor tags seen yet"
    elif not s["seen"]:
        text = "robot not seen"
    else:
        text = (f"x {s['x']:.1f}  y {s['y']:.1f}  hdg {s['heading']:.0f}  "
                f"edge {s['edge']:.1f} cm @ {s['edge_bearing']:.0f}")
        if s["held"]:
            text += "  (held)"
    if s["out"]:
        text += "  OUT (r = reset)"
    color = WHITE if s["seen"] and not s["out"] else RED
    cv2.putText(frame, f"{fps:.1f} fps", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2)

    # Mode at a glance: colored border plus a big label in the top-right corner
    auto = s["mode"] == "auto"
    mode_color = GREEN if auto else GRAY
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w - 1, h - 1), mode_color, 12)
    label = "AUTO" if auto else "IDLE"
    (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 4)
    cv2.putText(frame, label, (w - tw - 25, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.5, mode_color, 4)
    if behavior:  # what the board is doing, under the mode label
        (tw, _), _ = cv2.getTextSize(behavior, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        cv2.putText(frame, behavior, (w - tw - 25, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.9, WHITE, 2)
    cv2.putText(frame, text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=HERE / "config.yaml", type=Path)
    ap.add_argument("--calibration", default=HERE / "calibration.json", type=Path)
    ap.add_argument("--camera", type=int, help="webcam index (default: from config)")
    ap.add_argument("--mqtt", action="store_true", help="publish the state over MQTT")
    ap.add_argument("--broker", default=DEFAULT_BROKER, help="MQTT broker host")
    ap.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    if not args.calibration.exists():
        raise SystemExit(f"{args.calibration} not found, run calibrate.py first")
    floor, ring = load_calibration(args.calibration)
    m = cv2.moments(ring)
    ring_center = (m["m10"] / m["m00"], m["m01"] / m["m00"])
    finder = OpponentFinder(ring, floor, cfg["our_radius_cm"], cfg["opp_min_area_cm2"])
    board = {"behavior": None}
    mqtt_client = connect_mqtt(args.broker, args.port, board) if args.mqtt else None
    robot_id = cfg["robot_tag_id"]
    camera = args.camera if args.camera is not None else cfg["camera"]
    detector = make_detector()
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {camera}")
    w, h = cfg["camera_size"]
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print(f"Camera {camera}: asked {w}x{h}, got {got[0]}x{got[1]}")

    H = None
    mode = "idle"  # never start moving on launch
    out = False
    out_since = None  # when the robot was first seen past the edge in this stretch
    last_pose, last_pose_t = None, 0.0  # last real detection, for POSE_HOLD_S
    show_mask = False
    fps = 0.0
    last = time.perf_counter()
    last_print = 0.0
    frames = seen_frames = 0  # since the last print, for the robot seen rate
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                break

            tags = detect(detector, frame)
            new_H = homography(tags, floor)
            if new_H is not None:
                H = new_H
            now = time.perf_counter()
            pose = None
            if H is not None and robot_id in tags:
                pose = robot_pose(to_floor(H, tags[robot_id]), ring, ring_center,
                                  cfg["robot_heading_offset_deg"])
                last_pose, last_pose_t = pose, now
            # short dropout: keep the last pose so the tree and the opponent mask don't lose us
            held = pose is None and last_pose is not None and now - last_pose_t < POSE_HOLD_S
            if held:
                pose = last_pose
            opp = {"opp_seen": False}
            if H is not None:
                robot_xy = (pose["x"], pose["y"]) if pose else None
                opp = opponent_state(finder.find(frame, H, robot_xy), pose, ring)

            if pose is not None and not held:  # not seen: keep the timer as is
                out_since = (out_since or now) if pose["out"] else None
            out = out or (out_since is not None and now - out_since >= OUT_HOLD_S)
            state = {"t": round(time.time(), 3), "mode": mode, "seen": pose is not None,
                     "held": held, **(pose or {}), "out": out, **opp}
            if mqtt_client:
                mqtt_client.publish(TOPIC, json.dumps(state), qos=0)

            inst_fps = 1.0 / max(now - last, 1e-6)
            fps = 0.9 * fps + 0.1 * inst_fps if fps else inst_fps  # smoothed
            last = now
            frames += 1
            seen_frames += pose is not None and not held
            if now - last_print > 0.5:
                last_print = now
                seen = sorted(i for i in tags if i in floor)
                print(f"{fps:5.1f} fps  robot seen {100 * seen_frames // frames:3d}%  "
                      f"floor tags {seen}  {board['behavior'] or ''}  {state}")
                frames = seen_frames = 0

            draw(frame, tags, H, ring, robot_id, state, fps, board["behavior"])
            cv2.imshow("sumo track", frame)
            if show_mask:
                m = finder.mask
                cv2.imshow("opponent mask", cv2.resize(m, (m.shape[1] * 3, m.shape[0] * 3),
                                                       interpolation=cv2.INTER_NEAREST))
            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                mode = "idle" if mode == "auto" else "auto"
                print(f"mode: {mode}")
            elif key == ord("o"):
                show_mask = not show_mask
                if not show_mask:
                    cv2.destroyWindow("opponent mask")
            elif key == ord("r"):
                out, out_since = False, None
            elif key in (ord("q"), 27):
                break
    finally:
        if mqtt_client:
            # idle = the robot stops; nothing seen = no stale pose left behind
            final = {"t": round(time.time(), 3), "mode": "idle", "seen": False, "out": out,
                     "opp_seen": False}
            mqtt_client.publish(TOPIC, json.dumps(final), qos=0)
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
