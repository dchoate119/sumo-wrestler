"""Survey the floor tags and the ring, and save both to calibration.json.

Run with the floor tags down and the ring empty. The anchor tag plus its known
size gives an image -> floor homography; every other floor tag is mapped through
it. Each tag should then come out as a square of its known size. A check table
shows how close they get.

The ring is found in the top-down view: blue pixels (ring_hsv in config.yaml)
are cleaned up into the tape loop, and the hole inside it is the ring's inner
edge, saved as a polygon in floor cm. Keep tags off the tape so the loop closes.

    python laptop/calibrate.py
    python laptop/calibrate.py --camera 0 --secs 3

Preview keys: c = capture, q = quit.
Result keys: click the tape = resample its color, m = show mask,
s = save, r = redo, q = quit.
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
TOP_DOWN_PX = 900   # longest side of the top-down view
MAX_VIEW_CM = 500   # clip the top-down view to this far from the anchor

CLOSE_CM = 5        # bridge tape gaps up to about this wide
OPEN_CM = 1         # drop blue specks smaller than this
SIMPLIFY_CM = 1     # ring polygon may stray this far from the traced edge
MIN_RING_CM2 = 2500 # ignore holes smaller than this (50 x 50 cm)
SAMPLE_PX = 3       # click samples a (2 * SAMPLE_PX + 1) square patch
HUE_TOL = 10        # sampled hue +- this (OpenCV hue runs 0-179)


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


def top_down(frame, H):
    """Warp the whole frame to a bird's-eye view. Returns (view, origin_cm, px_per_cm)."""
    h, w = frame.shape[:2]
    c = to_floor(H, [[0, 0], [w, 0], [w, h], [0, h]])
    lo = np.clip(c.min(axis=0), -MAX_VIEW_CM, MAX_VIEW_CM)
    hi = np.clip(c.max(axis=0), -MAX_VIEW_CM, MAX_VIEW_CM)
    scale = TOP_DOWN_PX / (hi - lo).max()
    S = np.array([[scale, 0, -lo[0] * scale], [0, scale, -lo[1] * scale], [0, 0, 1]])
    size = tuple(((hi - lo) * scale).astype(int))
    return cv2.warpPerspective(frame, S @ H, size), lo, scale


def sample_hsv(view, pt):
    """HSV range around the median color of a small patch at pt."""
    x, y = pt
    patch = view[max(y - SAMPLE_PX, 0):y + SAMPLE_PX + 1, max(x - SAMPLE_PX, 0):x + SAMPLE_PX + 1]
    h, s, v = np.median(cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3), axis=0)
    lo = [max(h - HUE_TOL, 0), max(s - 80, 40), max(v - 100, 30)]
    hi = [min(h + HUE_TOL, 179), 255, 255]
    return [[int(n) for n in lo], [int(n) for n in hi]]


def find_ring(view, scale, hsv_range):
    """Inner edge of the tape loop. Returns (Nx2 polygon in view px or None, mask)."""
    lo, hi = (np.array(b, dtype=np.uint8) for b in hsv_range)
    mask = cv2.inRange(cv2.cvtColor(view, cv2.COLOR_BGR2HSV), lo, hi)

    def kernel(cm):
        n = max(int(cm * scale) | 1, 3)  # odd, at least 3 px
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel(OPEN_CM))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel(CLOSE_CM))

    # CCOMP: top level = outer edges of blue blobs, second level = their holes
    contours, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    holes = [c for c, h in zip(contours, hier[0]) if h[3] != -1] if contours else []
    if not holes:
        return None, mask
    hole = max(holes, key=cv2.contourArea)
    if cv2.contourArea(hole) < MIN_RING_CM2 * scale ** 2:
        return None, mask
    return cv2.approxPolyDP(hole, SIMPLIFY_CM * scale, True).reshape(-1, 2), mask


def draw_survey(view, floor, ring_cm, origin, scale):
    """Draw the floor tags (green) and ring (magenta) on the top-down view."""
    def px(pts):
        return ((np.asarray(pts) - origin) * scale).astype(int)

    for tag_id, c in floor.items():
        p = px(c)
        cv2.polylines(view, [p], True, (0, 255, 0), 2)
        cv2.circle(view, tuple(p[0]), 5, (0, 255, 0), -1)
        cv2.putText(view, f"id {tag_id}", tuple(p.min(axis=0) - [0, 8]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if ring_cm is not None:
        cv2.polylines(view, [px(ring_cm)], True, (255, 0, 255), 2)
    msg = "ring found" if ring_cm is not None else "NO RING - click the tape"
    cv2.putText(view, f"{msg}   click = sample   m = mask   s/r/q", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return view


def review(frame, H, floor, hsv_range):
    """Show the top-down view until a key decides. Returns (key, ring_cm, hsv_range)."""
    view, origin, scale = top_down(frame, H)
    clicks = []
    cv2.namedWindow("top-down")
    cv2.setMouseCallback("top-down", lambda ev, x, y, *_:
                         clicks.append((x, y)) if ev == cv2.EVENT_LBUTTONDOWN else None)
    show_mask = False
    ring_px, mask = find_ring(view, scale, hsv_range)
    while True:
        if clicks:
            hsv_range = sample_hsv(view, clicks.pop())
            print(f"ring_hsv: {hsv_range}")
            ring_px, mask = find_ring(view, scale, hsv_range)
        ring_cm = ring_px / scale + origin if ring_px is not None else None
        base = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask else view.copy()
        cv2.imshow("top-down", draw_survey(base, floor, ring_cm, origin, scale))
        key = cv2.waitKey(30) & 0xFF
        if key == ord("m"):
            show_mask = not show_mask
        elif key in (ord("s"), ord("r"), ord("q"), 27):
            cv2.destroyWindow("top-down")
            return key, ring_cm, hsv_range


def save(path, floor, ring_cm, hsv_range, cfg, frame_shape):
    data = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "image_size": [frame_shape[1], frame_shape[0]],
        "anchor_tag_id": cfg["anchor_tag_id"],
        "floor_tags": {
            str(i): {"size_cm": cfg["floor_tags"][i], "corners_cm": np.round(c, 3).tolist()}
            for i, c in sorted(floor.items())
        },
        "ring_hsv": hsv_range,
        "ring_cm": np.round(ring_cm, 2).tolist(),
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
    hsv_range = cfg["ring_hsv"]
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
            print("click the tape = resample color   m = mask   s = save   r = redo   q = quit")
            key, ring_cm, hsv_range = review(frame, H, floor, hsv_range)
            if key == ord("s"):
                if ring_cm is None:
                    print("No ring found, not saved. Capture again and click the tape.")
                    continue
                print(f"Ring: {len(ring_cm)} points, "
                      f"{cv2.contourArea(ring_cm.astype(np.float32)) / 1e4:.2f} m^2")
                save(args.out, floor, ring_cm, hsv_range, cfg, frame.shape)
                break
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
