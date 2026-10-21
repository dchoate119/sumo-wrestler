"""Offline check of the behavior tree with fake blackboards.

    python python/test_bt.py
"""

from bt import EDGE_CLEAR_CM, EDGE_CM, build_tree

AUTO = {"fresh": True, "mode": "auto", "seen": True}
NEAR = EDGE_CM - 5               # close enough to start an escape
HELD = (EDGE_CM + EDGE_CLEAR_CM) / 2  # past the start line, not yet clear
CLEAR = EDGE_CLEAR_CM + 5
MID = {"edge": CLEAR, "edge_bearing": 0, "center_bearing": 180}  # well inside the ring

# (description, blackboards ticked in order on one tree, expected behavior after each)
CASES = [
    ("no message yet", [{"fresh": False}], ["Idle"]),
    ("idle mode", [{"fresh": True, "mode": "idle"}], ["Idle"]),
    ("auto but stale", [{"fresh": False, "mode": "auto"}], ["Idle"]),
    ("auto", [{"fresh": True, "mode": "auto"}], ["Search"]),
    ("auto, not seen", [{**AUTO, "seen": False}], ["Search"]),
    ("middle of ring", [{**AUTO, "edge": CLEAR, "edge_bearing": 0, "center_bearing": 180}], ["Search"]),
    ("edge, facing it", [{**AUTO, "edge": NEAR, "edge_bearing": 10, "center_bearing": -170}], ["Edge escape"]),
    ("edge, facing away", [{**AUTO, "edge": NEAR, "edge_bearing": 170, "center_bearing": 10}], ["Search"]),
    ("past the edge", [{**AUTO, "edge": -3, "edge_bearing": -20, "center_bearing": 160}], ["Edge escape"]),
    ("edge but stale", [{**AUTO, "fresh": False, "edge": NEAR, "edge_bearing": 0, "center_bearing": 180}], ["Idle"]),
    ("out", [{**AUTO, "out": True, "edge": -20, "edge_bearing": 0, "center_bearing": 180}], ["Lost"]),
    ("out beats idle", [{"fresh": True, "mode": "idle", "out": True}], ["Lost"]),
    ("opp far, off to the side", [{**AUTO, **MID, "opp_seen": True, "opp_dist": 100, "opp_bearing": 90}], ["Chase"]),
    ("opp far, ahead", [{**AUTO, **MID, "opp_seen": True, "opp_dist": 100, "opp_bearing": 5}], ["Chase"]),
    ("opp close, off to the side", [{**AUTO, **MID, "opp_seen": True, "opp_dist": 25, "opp_bearing": 60}], ["Chase"]),
    ("opp close, ahead", [{**AUTO, **MID, "opp_seen": True, "opp_dist": 25, "opp_bearing": 5}], ["Attack"]),
    ("opp seen, we aren't", [{**AUTO, "seen": False, "opp_seen": True}], ["Search"]),
    ("edge beats attack", [{**AUTO, "edge": NEAR, "edge_bearing": 0, "center_bearing": 180,
                            "opp_seen": True, "opp_dist": 25, "opp_bearing": 0}], ["Edge escape"]),
    ("contact", [{**AUTO, **MID, "ultrasonic_cm": 5, "now": 10}], ["Flip"]),
    ("contact, no vision", [{**AUTO, "seen": False, "ultrasonic_cm": 5, "now": 10}], ["Flip"]),
    ("contact, cooling down", [{**AUTO, **MID, "ultrasonic_cm": 5, "now": 10, "last_flip": 9.5}], ["Search"]),
    ("contact, cooled down", [{**AUTO, **MID, "ultrasonic_cm": 5, "now": 10, "last_flip": 8}], ["Flip"]),
    ("too far to flip", [{**AUTO, **MID, "ultrasonic_cm": 30, "now": 10}], ["Search"]),
    ("no echo", [{**AUTO, **MID, "ultrasonic_cm": -1, "now": 10}], ["Search"]),
    ("contact but idle", [{"fresh": True, "mode": "idle", "ultrasonic_cm": 5, "now": 10}], ["Idle"]),
    ("edge beats flip", [{**AUTO, "edge": NEAR, "edge_bearing": 0, "center_bearing": 180,
                          "ultrasonic_cm": 5, "now": 10}], ["Edge escape"]),
    ("escape holds until clear", [
        {**AUTO, "edge": NEAR, "edge_bearing": 10, "center_bearing": -170},   # starts
        {**AUTO, "edge": NEAR, "edge_bearing": 170, "center_bearing": 10},    # turned away: holds
        {**AUTO, "seen": False},                                               # dropout
        {**AUTO, "edge": HELD, "edge_bearing": 180, "center_bearing": 0},     # back, still holds
        {**AUTO, "edge": CLEAR, "edge_bearing": 180, "center_bearing": 0},    # clear: done
    ], ["Edge escape", "Edge escape", "Search", "Edge escape", "Search"]),
    ("teleop", [{**AUTO, **MID, "teleop": {"l": 150, "r": 150}}], ["Teleop"]),
    ("teleop hold", [{**AUTO, **MID, "teleop": {"l": 0, "r": 0}}], ["Teleop"]),
    ("teleop, not seen", [{**AUTO, "seen": False, "teleop": {"l": 150, "r": 150}}], ["Teleop"]),
    ("idle beats teleop", [{"fresh": True, "mode": "idle", "teleop": {"l": 150, "r": 150}}], ["Idle"]),
    ("out beats teleop", [{**AUTO, "out": True, "teleop": {"l": 150, "r": 150}}], ["Lost"]),
    ("teleop beats edge", [{**AUTO, "edge": NEAR, "edge_bearing": 0, "center_bearing": 180,
                            "teleop": {"l": 150, "r": 150}}], ["Teleop"]),
    ("teleop beats attack", [{**AUTO, **MID, "opp_seen": True, "opp_dist": 25, "opp_bearing": 0,
                              "teleop": {"l": 0, "r": 0}}], ["Teleop"]),
    ("teleop let go", [{**AUTO, **MID, "teleop": {"l": 150, "r": 150}}, {**AUTO, **MID}],
     ["Teleop", "Search"]),
]

# Edge guard: (description, blackboard, expected drive command)
NEAR_EDGE = {**AUTO, "edge": NEAR, "center_bearing": 180}
GUARD_CASES = [
    ("mid ring, forward", {**AUTO, **MID, "teleop": {"l": 150, "r": 150}}, ("drive", 150, 150)),
    ("edge ahead, forward", {**NEAR_EDGE, "edge_bearing": 0, "teleop": {"l": 150, "r": 150}}, ("drive", 0, 0)),
    ("edge ahead, arc", {**NEAR_EDGE, "edge_bearing": 0, "teleop": {"l": 250, "r": 50}}, ("drive", 100, -100)),
    ("edge ahead, back up", {**NEAR_EDGE, "edge_bearing": 0, "teleop": {"l": -150, "r": -150}}, ("drive", -150, -150)),
    ("edge behind, back up", {**NEAR_EDGE, "edge_bearing": 180, "teleop": {"l": -150, "r": -150}}, ("drive", 0, 0)),
    ("edge behind, forward", {**NEAR_EDGE, "edge_bearing": 180, "teleop": {"l": 150, "r": 150}}, ("drive", 150, 150)),
    ("edge ahead, spin", {**NEAR_EDGE, "edge_bearing": 0, "teleop": {"l": 100, "r": -100}}, ("drive", 100, -100)),
    ("not seen, forward", {**AUTO, "seen": False, "teleop": {"l": 150, "r": 150}}, ("drive", 150, 150)),
]

failed = 0
for desc, bbs, expected in CASES:
    tree = build_tree()  # fresh tree: some nodes remember earlier ticks
    got = []
    for bb in bbs:
        tree.tick(bb)
        got.append(bb.get("behavior"))
    ok = got == expected
    failed += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {desc:<28} -> {' > '.join(got)} {bb.get('cmd')}")
for desc, bb, expected in GUARD_CASES:
    build_tree().tick(bb)
    ok = bb["cmd"] == expected
    failed += not ok
    print(f"{'ok  ' if ok else 'FAIL'} guard: {desc:<21} -> {bb['cmd']}")
total = len(CASES) + len(GUARD_CASES)
print(f"{total - failed}/{total} passed")
