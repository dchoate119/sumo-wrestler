"""Survey the floor tags and save their floor positions to calibration.json.

Run with the floor tags down and the ring empty. The anchor tag plus its known
size gives an image -> floor homography; every other floor tag is mapped through
it. Each tag should then come out as a square of its known size. A check table
shows how close they get.

    python laptop/calibrate.py
    python laptop/calibrate.py --camera 0 --secs 3

Preview keys: c = capture, q = quit. Result keys: s = save, r = redo, q = quit.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from geometry import detect, make_detector, tag_square, to_floor

HERE = Path(__file__).parent

EDGE_TOL_CM = 0.5   # flag a tag if any edge is off its known size by more than this
ANGLE_TOL_DEG = 3   # ... or any corner is off 90 degrees by more than this
TOP_DOWN_PX = 800   # longest side of the top-down view
MARGIN_CM = 20      # padding around the tags in the top-down view


def capture(cap, detector, secs):
    """Average each tag's corners over secs. Returns ({id: corners}, {id: count}, frames)."""
    seen = {}
    frames = 0
    end = time.perf_counter() + secs
    while time.perf_counter() < end:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit("Camera read failed")
        frames += 1
        for tag_id, c in detect(detector, frame).items():
            seen.setdefault(tag_id, []).append(c)
    avg = {i: np.mean(cs, axis=0) for i, cs in seen.items()}
    counts = {i: len(cs) for i, cs in seen.items()}
    return avg, counts, frames


def survey(avg, cfg):
    """Homography from the anchor tag, then every floor tag's corners in floor cm."""
    anchor = cfg["anchor_tag_id"]
    src = avg[anchor].astype(np.float32)
    dst = tag_square(cfg["floor_tags"][anchor]).astype(np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    floor = {i: to_floor(H, avg[i]) for i in cfg["floor_tags"] if i in avg}
    return H, floor


def check(floor, cfg):
    """Print each tag's edge lengths and corner angles. Returns True if all are within tolerance."""
    anchor_center = floor[cfg["anchor_tag_id"]].mean(axis=0)
    all_ok = True
    print(f"\n{'id':>4} {'dist cm':>8} {'edges cm (min-max)':>20} {'angles deg (min-max)':>22}")
    for tag_id, c in sorted(floor.items()):
        size = cfg["floor_tags"][tag_id]
        edges = np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1)
        to_prev = np.roll(c, 1, axis=0) - c
        to_next = np.roll(c, -1, axis=0) - c
        cos = np.sum(to_prev * to_next, axis=1) / (
            np.linalg.norm(to_prev, axis=1) * np.linalg.norm(to_next, axis=1))
        angles = np.degrees(np.arccos(np.clip(cos, -1, 1)))
        ok = (np.abs(edges - size).max() <= EDGE_TOL_CM
              and np.abs(angles - 90).max() <= ANGLE_TOL_DEG)
        all_ok &= ok
        dist = np.linalg.norm(c.mean(axis=0) - anchor_center)
        print(f"{tag_id:>4} {dist:8.1f} {edges.min():9.2f} - {edges.max():<8.2f} "
              f"{angles.min():10.1f} - {angles.max():<8.1f} {'ok' if ok else 'CHECK'}")
    print(f"(each tag should be {EDGE_TOL_CM} cm / {ANGLE_TOL_DEG} deg from its known square)")
    return all_ok


def top_down(frame, H, floor):
    """Warp the frame to a bird's-eye view covering all floor tags, with tags drawn."""
    pts = np.vstack(list(floor.values()))
    x0, y0 = pts.min(axis=0) - MARGIN_CM
    x1, y1 = pts.max(axis=0) + MARGIN_CM
    scale = TOP_DOWN_PX / max(x1 - x0, y1 - y0)  # px per cm
    S = np.array([[scale, 0, -x0 * scale], [0, scale, -y0 * scale], [0, 0, 1]])
    size = (int((x1 - x0) * scale), int((y1 - y0) * scale))
    view = cv2.warpPerspective(frame, S @ H, size)

    for tag_id, c in floor.items():
        px = ((c - [x0, y0]) * scale).astype(int)
        cv2.polylines(view, [px], True, (0, 255, 0), 2)
        cv2.circle(view, tuple(px[0]), 5, (0, 255, 0), -1)
        cv2.putText(view, f"id {tag_id}", tuple(px.min(axis=0) - [0, 8]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return view


def save(path, floor, cfg, frame_shape):
    data = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "image_size": [frame_shape[1], frame_shape[0]],
        "anchor_tag_id": cfg["anchor_tag_id"],
        "floor_tags": {
            str(i): {"size_cm": cfg["floor_tags"][i], "corners_cm": np.round(c, 3).tolist()}
            for i, c in sorted(floor.items())
        },
    }
    path.write_text(json.dumps(data, indent=2))
    print(f"Saved {path}")


def draw_preview(frame, tags, cfg):
    for tag_id, c in tags.items():
        color = (0, 255, 0) if tag_id in cfg["floor_tags"] else (0, 0, 255)
        cv2.polylines(frame, [c.astype(int)], True, color, 2)
        cv2.putText(frame, f"id {tag_id}", tuple(c.min(axis=0).astype(int) - [0, 8]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(frame, "c = capture   q = quit", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=HERE / "config.yaml", type=Path)
    ap.add_argument("--camera", type=int, help="webcam index (default: from config)")
    ap.add_argument("--secs", type=float, default=1.5, help="capture duration")
    ap.add_argument("--out", default=HERE / "calibration.json", type=Path)
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    camera = args.camera if args.camera is not None else cfg["camera"]
    detector = make_detector()
    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {camera}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("Camera read failed")
            draw_preview(frame, detect(detector, frame), cfg)
            cv2.imshow("calibrate", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key != ord("c"):
                continue

            print(f"Capturing {args.secs} s ...")
            avg, counts, frames = capture(cap, detector, args.secs)
            missing = [i for i in cfg["floor_tags"] if i not in avg]
            if missing:
                print(f"Not seen: floor tags {missing}")
            for i in cfg["floor_tags"]:
                if i in counts and counts[i] < frames / 2:
                    print(f"Tag {i} seen in only {counts[i]}/{frames} frames")
            if cfg["anchor_tag_id"] not in avg:
                print(f"Anchor tag {cfg['anchor_tag_id']} not seen, try again")
                continue

            H, floor = survey(avg, cfg)
            check(floor, cfg)
            ok, frame = cap.read()
            cv2.imshow("top-down", top_down(frame, H, floor))
            print("s = save   r = redo   q = quit")
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyWindow("top-down")
            if key == ord("s"):
                save(args.out, floor, cfg, frame.shape)
                break
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
