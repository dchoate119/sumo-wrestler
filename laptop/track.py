"""Live tracker: our robot's pose and the opponent's position, in floor cm.

Needs calibration.json from calibrate.py. Every frame, the floor tags in view
give a fresh image -> floor homography (the last good one is reused if none are
seen), and the robot tag is mapped through it. The opponent is found by
opponent.py.

    x, y         robot tag center in floor cm (see geometry.py for the frame)
    heading      direction of the tag's top edge, degrees from +x toward +y
    edge         distance from the center to the ring's inner edge, cm (negative = past it)
    out          all 4 robot tag corners are past the inner edge
    opp_x, opp_y opponent blob center in floor cm
    opp_edge     opponent's distance to the inner edge, cm
    opp_dist     cm from us to the opponent         (only when we are seen too)
    opp_bearing  direction to the opponent, degrees from our heading, + toward +y side

The robot tag sits above the floor, so its position is skewed slightly away
from the point under the camera. Fine for edge distances of a few cm.

    python laptop/track.py
    python laptop/track.py --camera 0

Keys in the preview window: o = show opponent mask, q (or Esc) = quit.
"""

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from geometry import bearing, detect, make_detector, to_floor
from opponent import OpponentFinder

HERE = Path(__file__).parent

GREEN = (0, 255, 0)
ORANGE = (0, 165, 255)
MAGENTA = (255, 0, 255)
RED = (0, 0, 255)
WHITE = (255, 255, 255)


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


def robot_pose(corners_cm, ring):
    """Pose dict from the robot tag's corners in floor cm."""
    x, y = corners_cm.mean(axis=0)
    front = corners_cm[:2].mean(axis=0) - corners_cm[2:].mean(axis=0)  # bottom edge -> top edge
    edge = cv2.pointPolygonTest(ring, (float(x), float(y)), True)  # + inside, - outside
    out = all(cv2.pointPolygonTest(ring, (float(cx), float(cy)), False) < 0
              for cx, cy in corners_cm)
    return {
        "x": round(float(x), 1),
        "y": round(float(y), 1),
        "heading": round(math.degrees(math.atan2(front[1], front[0])), 1),
        "edge": round(float(edge), 1),
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


def draw(frame, tags, H, ring, robot_id, pose, opp, fps):
    if H is not None:
        ring_px = to_floor(np.linalg.inv(H), ring.reshape(-1, 2))
        cv2.polylines(frame, [ring_px.astype(int)], True, MAGENTA, 2)
    for tag_id, c in tags.items():
        color = ORANGE if tag_id == robot_id else GREEN
        cv2.polylines(frame, [c.astype(int)], True, color, 2)
    if robot_id in tags:
        c = tags[robot_id]
        center = c.mean(axis=0)
        cv2.arrowedLine(frame, tuple(center.astype(int)), tuple(c[:2].mean(axis=0).astype(int)),
                        ORANGE, 3, tipLength=0.3)
    if opp["opp_seen"]:
        p = tuple(to_floor(np.linalg.inv(H), [[opp["opp_x"], opp["opp_y"]]])[0].astype(int))
        cv2.drawMarker(frame, p, RED, cv2.MARKER_CROSS, 30, 3)
        label = f"opp  edge {opp['opp_edge']:.0f}"
        if "opp_dist" in opp:
            label += f"  {opp['opp_dist']:.0f} cm @ {opp['opp_bearing']:.0f}"
        cv2.putText(frame, label, (p[0] + 15, p[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)

    if H is None:
        text, color = "no floor tags seen yet", RED
    elif pose is None:
        text, color = "robot not seen", RED
    else:
        text = (f"x {pose['x']:.1f}  y {pose['y']:.1f}  hdg {pose['heading']:.0f}  "
                f"edge {pose['edge']:.1f} cm")
        color = RED if pose["out"] else WHITE
        if pose["out"]:
            text += "  OUT"
    cv2.putText(frame, f"{fps:.1f} fps", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2)
    cv2.putText(frame, text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=HERE / "config.yaml", type=Path)
    ap.add_argument("--calibration", default=HERE / "calibration.json", type=Path)
    ap.add_argument("--camera", type=int, help="webcam index (default: from config)")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    if not args.calibration.exists():
        raise SystemExit(f"{args.calibration} not found, run calibrate.py first")
    floor, ring = load_calibration(args.calibration)
    finder = OpponentFinder(ring, floor, cfg["our_radius_cm"], cfg["opp_min_area_cm2"])
    robot_id = cfg["robot_tag_id"]
    camera = args.camera if args.camera is not None else cfg["camera"]
    detector = make_detector()
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {camera}")

    H = None
    show_mask = False
    fps = 0.0
    last = time.perf_counter()
    last_print = 0.0
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
            pose = None
            if H is not None and robot_id in tags:
                pose = robot_pose(to_floor(H, tags[robot_id]), ring)
            opp = {"opp_seen": False}
            if H is not None:
                robot_xy = (pose["x"], pose["y"]) if pose else None
                opp = opponent_state(finder.find(frame, H, robot_xy), pose, ring)

            now = time.perf_counter()
            inst_fps = 1.0 / max(now - last, 1e-6)
            fps = 0.9 * fps + 0.1 * inst_fps if fps else inst_fps  # smoothed
            last = now
            if now - last_print > 0.5:
                last_print = now
                seen = sorted(i for i in tags if i in floor)
                print(f"{fps:5.1f} fps  floor tags {seen}  robot {pose or 'not seen'}  {opp}")

            draw(frame, tags, H, ring, robot_id, pose, opp, fps)
            cv2.imshow("sumo track", frame)
            if show_mask:
                m = finder.mask
                cv2.imshow("opponent mask", cv2.resize(m, (m.shape[1] * 3, m.shape[0] * 3),
                                                       interpolation=cv2.INTER_NEAREST))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("o"):
                show_mask = not show_mask
                if not show_mask:
                    cv2.destroyWindow("opponent mask")
            elif key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
