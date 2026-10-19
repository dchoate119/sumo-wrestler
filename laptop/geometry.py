"""Shared helpers for the sumo vision scripts: tag detection and floor geometry.

Floor frame: the anchor tag's corner 0 is the origin, x runs along its top edge
and y along its side edge, in cm. Same handedness as the image (y "down").
"""

import math

import cv2
import numpy as np


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def detect(detector, frame):
    """Return {id: 4x2 corners in pixels}. Corner 0 is the tag's own top-left, then clockwise."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    # reshape: shapes differ across OpenCV versions
    return {int(i): c.reshape(-1, 2).astype(np.float64) for c, i in zip(corners, ids.flatten())}


def tag_square(size_cm):
    """A tag's 4 corners in its own frame, in the same order detect() returns them."""
    s = size_cm
    return np.array([[0, 0], [s, 0], [s, s], [0, s]], dtype=np.float64)


def to_floor(H, pts):
    """Map Nx2 image points to floor cm through homography H (image -> floor)."""
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def nearest_on_polygon(poly, pt):
    """Closest point to pt on the edge of closed polygon poly (Nx2)."""
    poly = np.asarray(poly, dtype=np.float64).reshape(-1, 2)
    a, ab = poly, np.roll(poly, -1, axis=0) - poly
    t = np.clip(np.sum((pt - a) * ab, axis=1) / np.maximum(np.sum(ab * ab, axis=1), 1e-9), 0, 1)
    proj = a + t[:, None] * ab
    return proj[np.argmin(np.linalg.norm(proj - pt, axis=1))]


def bearing(x, y, heading, tx, ty):
    """Direction to (tx, ty) relative to heading, degrees in [-180, 180). 0 = dead ahead."""
    a = math.degrees(math.atan2(ty - y, tx - x)) - heading
    return (a + 180) % 360 - 180
