"""Opponent finder: the biggest thing inside the ring that isn't rug or us.

Works on a coarse top-down view of the ring (1 px per cm). The rug color is
fit as a smooth gradient (a plane in LAB over x, y) so uneven lighting doesn't
count as an object; robots are dropped from the fit as outliers, so they can sit
in the ring the whole time. Pixels far from the fit are objects; the threshold
adapts to how noisy the rug is. Floor tags, the tape, and a circle around our
own tag are masked out, and the biggest blob left is the opponent.
"""

import time

import cv2
import numpy as np

PX_PER_CM = 1
BLUR_PX = 5          # smooths out rug texture (odd, in px)
EDGE_TRIM_CM = 2     # ignore this much just inside the ring's inner edge
TAG_MARGIN_CM = 2    # ... and around each floor tag
SPREAD_K = 4         # object = color distance > median + SPREAD_K * MAD of the rug
MIN_DIFF = 12        # never use a threshold below this (LAB units)
FLOOR_UPDATE_S = 1.0 # re-fit the rug color this often
FIT_STEP = 3         # fit on every FIT_STEP-th rug pixel
FIT_ROUNDS = 3       # fit, drop outliers (robots), refit
CLEAN_PX = 3         # open/close kernel for the object mask


def kernel(n):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))


class OpponentFinder:
    def __init__(self, ring_cm, floor_tags, our_radius_cm, min_area_cm2):
        """ring_cm: Nx2 inner edge in floor cm. floor_tags: {id: 4x2 corners in floor cm}."""
        ring_cm = np.asarray(ring_cm, dtype=np.float64).reshape(-1, 2)
        self.origin = ring_cm.min(axis=0) - 1
        self.size = tuple(np.ceil((ring_cm.max(axis=0) + 1 - self.origin) * PX_PER_CM).astype(int))
        s = PX_PER_CM
        self.S = np.array([[s, 0, -self.origin[0] * s], [0, s, -self.origin[1] * s], [0, 0, 1]])

        # Where to look: inside the ring, minus a strip along the edge and the floor tags
        area = np.zeros(self.size[::-1], np.uint8)
        cv2.fillPoly(area, [self._px(ring_cm)], 255)
        area = cv2.erode(area, kernel(2 * EDGE_TRIM_CM * s + 1))
        for c in floor_tags.values():
            cv2.fillPoly(area, [self._px(c)], 0)
            cv2.polylines(area, [self._px(c)], True, 0, 2 * TAG_MARGIN_CM * s)
        self.area = area

        self.our_radius_px = int(our_radius_cm * s)
        self.min_area_px = min_area_cm2 * s * s
        ys, xs = np.mgrid[0:self.size[1], 0:self.size[0]]
        self.grid = np.dstack([xs, ys, np.ones_like(xs)]).astype(np.float32)  # [x, y, 1] per pixel
        self.coef = None  # 3x3: grid @ coef = rug LAB
        self.threshold = None
        self.last_update = 0.0
        self.mask = np.zeros_like(area)  # last object mask, for debugging

    def _px(self, pts_cm):
        return np.round((np.asarray(pts_cm) - self.origin) * PX_PER_CM).astype(np.int32)

    def _fit_floor(self, lab, look):
        """Fit rug LAB = [x, y, 1] @ coef, dropping outliers. Returns False if nothing to fit."""
        sel = look[::FIT_STEP, ::FIT_STEP] > 0
        A = self.grid[::FIT_STEP, ::FIT_STEP][sel]
        pix = lab[::FIT_STEP, ::FIT_STEP][sel]
        if len(pix) < 3:
            return False
        keep = np.ones(len(pix), bool)
        for _ in range(FIT_ROUNDS):
            coef, *_ = np.linalg.lstsq(A[keep], pix[keep], rcond=None)
            d = np.linalg.norm(pix - A @ coef, axis=1)
            med = np.median(d[keep])
            threshold = max(med + SPREAD_K * np.median(np.abs(d[keep] - med)), MIN_DIFF)
            keep = d < threshold
        self.coef, self.threshold = coef, threshold
        return True

    def find(self, frame, H, robot_xy=None):
        """Opponent (x, y) in floor cm, or None. robot_xy: our tag center, masked out."""
        view = cv2.warpPerspective(frame, self.S @ H, self.size)
        view = cv2.GaussianBlur(view, (BLUR_PX, BLUR_PX), 0)
        lab = cv2.cvtColor(view, cv2.COLOR_BGR2LAB).astype(np.float32)

        look = self.area.copy()
        if robot_xy is not None:
            cv2.circle(look, tuple(self._px(robot_xy)), self.our_radius_px, 0, -1)

        now = time.perf_counter()
        if self.coef is None or now - self.last_update > FLOOR_UPDATE_S:
            if not self._fit_floor(lab, look):
                return None
            self.last_update = now

        dist = np.linalg.norm(lab - self.grid @ self.coef, axis=2)
        mask = ((dist > self.threshold) & (look > 0)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel(CLEAN_PX))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel(CLEAN_PX))
        self.mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        blob = max(contours, key=cv2.contourArea)
        if cv2.contourArea(blob) < self.min_area_px:
            return None
        m = cv2.moments(blob)
        center_px = np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])
        x, y = center_px / PX_PER_CM + self.origin
        return float(x), float(y)
