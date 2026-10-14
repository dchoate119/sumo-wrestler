"""Make a printable PDF of the AprilTags (tag36h11) listed in config.yaml.

One tag per US Letter page, with the black square at its exact size from the
config. Print at 100% / "Actual size" (not "Fit to page"), then measure a black
square to check the scale.

    python laptop/make_tags.py               # writes laptop/tags.pdf
"""

import argparse
from pathlib import Path

import cv2
import yaml
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).parent
DPI = 300
PAGE_PX = (int(8.5 * DPI), int(11 * DPI))  # US Letter
TAG_MODULES = 8  # tag36h11 black square: 6x6 data bits + 1-bit black border


def cm_to_px(cm):
    return round(cm / 2.54 * DPI)


def tag_image(dictionary, tag_id, size_cm):
    """Tag as a PIL image whose black square is exactly size_cm wide."""
    bits = cv2.aruco.generateImageMarker(dictionary, tag_id, TAG_MODULES, borderBits=1)
    px = cm_to_px(size_cm)
    return Image.fromarray(bits).resize((px, px), Image.NEAREST)


def page_for(dictionary, tag_id, size_cm, role, font):
    page = Image.new("L", PAGE_PX, 255)
    tag = tag_image(dictionary, tag_id, size_cm)
    x = (PAGE_PX[0] - tag.width) // 2
    y = cm_to_px(2.5)
    page.paste(tag, (x, y))

    draw = ImageDraw.Draw(page)
    below = y + tag.height + cm_to_px(1.5)
    draw.text((x, below), f"tag36h11  id {tag_id}  ({role})", fill=0, font=font)
    draw.text((x, below + 70), f"black square = {size_cm:g} cm", fill=0, font=font)
    return page


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=HERE / "config.yaml", type=Path)
    ap.add_argument("--out", default=HERE / "tags.pdf", type=Path)
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    tags = [(cfg["robot_tag_id"], cfg["robot_tag_size_cm"], "robot")]
    for tag_id, size in cfg["floor_tags"].items():
        role = "floor, anchor" if tag_id == cfg["anchor_tag_id"] else "floor"
        tags.append((tag_id, size, role))

    usable_cm = PAGE_PX[0] / DPI * 2.54 - 2 * 2.0  # leave ~2 cm white border each side
    for tag_id, size, _ in tags:
        if size > usable_cm:
            raise SystemExit(f"Tag {tag_id} at {size} cm won't fit on Letter (max ~{usable_cm:.1f} cm)")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    font = ImageFont.load_default(size=50)
    pages = [page_for(dictionary, tag_id, size, role, font) for tag_id, size, role in tags]
    pages[0].save(args.out, save_all=True, append_images=pages[1:], resolution=DPI)
    print(f"Wrote {len(pages)} pages to {args.out}. Print at 100% / Actual size.")


if __name__ == "__main__":
    main()
