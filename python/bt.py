"""Behavior tree: small node classes and the sumo tree.

Every tick gets a blackboard dict (the latest state message plus board-side
fields like fresh). Nodes return SUCCESS, FAILURE or RUNNING. Actions don't
touch hardware: they leave a command in bb["cmd"] and their name in
bb["behavior"], and main.py sends the command. That keeps the tree runnable off
the board (see test_bt.py).
"""

SUCCESS, FAILURE, RUNNING = "success", "failure", "running"

SEARCH_SPEED = 80  # of 255, each wheel opposite: spin in place


class Selector:
    """Tries children in order; the first that doesn't fail wins."""

    def __init__(self, *children):
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status != FAILURE:
                return status
        return FAILURE


class Sequence:
    """Runs children in order; stops at the first that doesn't succeed."""

    def __init__(self, *children):
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status != SUCCESS:
                return status
        return SUCCESS


class Condition:
    def __init__(self, name, fn):
        self.name, self.fn = name, fn

    def tick(self, bb):
        return SUCCESS if self.fn(bb) else FAILURE


class Action:
    """fn(bb) returns the command, e.g. ("drive", left, right)."""

    def __init__(self, name, fn):
        self.name, self.fn = name, fn

    def tick(self, bb):
        bb["cmd"] = self.fn(bb)
        bb["behavior"] = self.name
        return RUNNING


def build_tree():
    return Selector(
        # stale counts as idle, so the robot stops if the laptop goes quiet
        Sequence(Condition("not auto", lambda bb: bb.get("mode") != "auto" or not bb["fresh"]),
                 Action("Idle", lambda bb: ("drive", 0, 0))),
        Action("Search", lambda bb: ("drive", SEARCH_SPEED, -SEARCH_SPEED)),
    )
