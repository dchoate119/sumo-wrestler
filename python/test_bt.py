"""Offline check of the behavior tree with fake blackboards.

    python python/test_bt.py
"""

from bt import EDGE_CLEAR_CM, EDGE_CM, build_tree

AUTO = {"fresh": True, "mode": "auto", "seen": True}
NEAR = EDGE_CM - 5               # close enough to start an escape
HELD = (EDGE_CM + EDGE_CLEAR_CM) / 2  # past the start line, not yet clear
CLEAR = EDGE_CLEAR_CM + 5

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
    ("escape holds until clear", [
        {**AUTO, "edge": NEAR, "edge_bearing": 10, "center_bearing": -170},   # starts
        {**AUTO, "edge": NEAR, "edge_bearing": 170, "center_bearing": 10},    # turned away: holds
        {**AUTO, "seen": False},                                               # dropout
        {**AUTO, "edge": HELD, "edge_bearing": 180, "center_bearing": 0},     # back, still holds
        {**AUTO, "edge": CLEAR, "edge_bearing": 180, "center_bearing": 0},    # clear: done
    ], ["Edge escape", "Edge escape", "Search", "Edge escape", "Search"]),
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
    print(f"{'ok  ' if ok else 'FAIL'} {desc:<24} -> {' > '.join(got)} {bb.get('cmd')}")
print(f"{len(CASES) - failed}/{len(CASES)} passed")
