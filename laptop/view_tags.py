"""Live view of the AprilTags (tag36h11) the camera can see.

Outlines each tag and labels it with its ID and side length in pixels, to check
the tags are big enough to detect from the camera's height (aim for 25+ px).
Green = floor tag, orange = robot tag, red = ID not in config.yaml.

    python laptop/view_tags.py               # camera from config.yaml
    python laptop/view_tags.py --camera 0    # override

Press q (or Esc) in the preview window to quit.
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).parent

GREEN = (0, 255, 0)
ORANGE = (0, 165, 255)
RED = (0, 0, 255)


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def tag_color(tag_id, cfg):
    if tag_id == cfg["robot_tag_id"]:
        return ORANGE
    if tag_id in cfg["floor_tags"]:
        return GREEN
    return RED


def side_px(corners):
    """Mean of the tag's 4 edge lengths, in pixels."""
    return float(np.mean(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=HERE / "config.yaml", type=Path)
    ap.add_argument("--camera", type=int, help="webcam index (default: from config)")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    camera = args.camera if args.camera is not None else cfg["camera"]
    detector = make_detector()

    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {camera}")
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {camera}: {w}x{h}")

    fps = 0.0
    last = time.perf_counter()
    last_print = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            found = []
            if ids is not None:
                for c, tag_id in zip(corners, ids.flatten()):
                    c = c.reshape(-1, 2)  # shapes differ across OpenCV versions
                    tag_id = int(tag_id)
                    side = side_px(c)
                    found.append((tag_id, side))

                    color = tag_color(tag_id, cfg)
                    cv2.polylines(frame, [c.astype(int)], True, color, 2)
                    cv2.circle(frame, tuple(c[0].astype(int)), 5, color, -1)  # corner 0 shows orientation
                    x, y = c.min(axis=0).astype(int)
                    cv2.putText(frame, f"id {tag_id}  {side:.0f} px", (x, max(y - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            now = time.perf_counter()
            inst_fps = 1.0 / max(now - last, 1e-6)
            fps = 0.9 * fps + 0.1 * inst_fps if fps else inst_fps  # smoothed
            last = now

            if now - last_print > 0.5:
                last_print = now
                tags = " ".join(f"{i}({s:.0f}px)" for i, s in found) or "none"
                print(f"{fps:5.1f} fps  tags: {tags}")

            cv2.putText(frame, f"{fps:.1f} fps  {w}x{h}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("sumo tags", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
