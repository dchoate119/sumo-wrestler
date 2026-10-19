"""Offline check of the behavior tree with fake blackboards.

    python python/test_bt.py
"""

from bt import build_tree

# (description, blackboard, expected behavior)
CASES = [
    ("no message yet", {"fresh": False}, "Idle"),
    ("idle mode", {"fresh": True, "mode": "idle"}, "Idle"),
    ("auto but stale", {"fresh": False, "mode": "auto"}, "Idle"),
    ("auto", {"fresh": True, "mode": "auto"}, "Search"),
]

tree = build_tree()
failed = 0
for desc, bb, expected in CASES:
    tree.tick(bb)
    ok = bb.get("behavior") == expected
    failed += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {desc:<16} -> {bb.get('behavior')} {bb.get('cmd')}")
print(f"{len(CASES) - failed}/{len(CASES)} passed")
