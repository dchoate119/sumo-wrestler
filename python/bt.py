"""Behavior tree: small node classes and the sumo tree.

Every tick gets a blackboard dict (the latest state message plus board-side
fields like fresh). Nodes return SUCCESS, FAILURE or RUNNING. Actions don't
touch hardware: they leave a command in bb["cmd"] and their name in
bb["behavior"], and main.py sends the command. That keeps the tree runnable off
the board (see test_bt.py).
"""

import math

SUCCESS, FAILURE, RUNNING = "success", "failure", "running"

SEARCH_SPEED = 80     # of 255, each wheel opposite: spin in place
EDGE_CM = 20          # escape when the tag center is closer than this to the edge
FACING_EDGE_DEG = 90  # ... and the edge is within this of our heading
EDGE_CLEAR_CM = EDGE_CM + 20  # once escaping, keep going until this far in
ESCAPE_SPEED = 120    # forward speed once pointed at the center
TURN_SPEED = 90       # pivot speed while pointing at the center
ALIGN_DEG = 30        # pivot in place until the target is within this
STEER_PER_DEG = 2     # while driving, wheel speed difference per degree off
CHASE_SPEED = 100     # forward speed toward the opponent once pointed at it
ATTACK_SPEED = 255    # full push
ATTACK_DEG = 15       # attack when the opponent is within this of our heading
ATTACK_CM = 40        # ... and its center is closer than this to ours (touching is ~15-20)
FLIP_CM = 18          # ultrasonic reading (from the sensor) that counts as contact; it reads ~15 at a push
FLIP_COOLDOWN_S = 1.5 # one flip takes about 1 s; lego_bridge.py ignores flips mid-swing anyway
# Bearings grow from +x toward +y, which looks clockwise from the camera, so a
# positive bearing means turn right. Flip this if the robot turns away instead.
TURN_SIGN = -1


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


class EdgeNear:
    """Starts an escape when near the edge and facing it, then holds until edge > EDGE_CLEAR_CM.

    Holding without the facing check keeps Search from spinning the robot back
    toward the edge halfway through the turn. A dropout (not seen) fails this
    tick but keeps the latch, so the escape resumes when the tag comes back.
    """

    def __init__(self):
        self.active = False

    def tick(self, bb):
        if not bb.get("seen"):
            return FAILURE
        if self.active:
            self.active = bb["edge"] < EDGE_CLEAR_CM
        else:
            self.active = bb["edge"] < EDGE_CM and abs(bb["edge_bearing"]) < FACING_EDGE_DEG
        return SUCCESS if self.active else FAILURE


def turn_and_drive(bearing, speed):
    """Pivot toward bearing (degrees, + = right) until roughly aligned, then drive and steer."""
    if abs(bearing) > ALIGN_DEG:
        turn = TURN_SIGN * (TURN_SPEED if bearing > 0 else -TURN_SPEED)
        return ("drive", turn, -turn)
    steer = TURN_SIGN * STEER_PER_DEG * bearing
    return ("drive", speed + steer, speed - steer)


def teleop_drive(bb):
    """Pass the operator's (l, r) through, minus any forward motion toward the edge.

    The edge guard is the shared-control part: near the edge, driving toward it
    (forward with the edge ahead, or back with it behind) loses its forward
    component, so turning and backing away still work. A flip request uses the
    same cooldown as the Flip behavior.
    """
    t = bb["teleop"]
    l, r = t.get("l", 0), t.get("r", 0)
    fwd = (l + r) / 2
    if (bb.get("seen") and bb["edge"] < EDGE_CM
            and fwd * math.cos(math.radians(bb["edge_bearing"])) > 0):
        l, r = l - fwd, r - fwd
    if t.get("flip") and bb["now"] - bb.get("last_flip", -1e9) >= FLIP_COOLDOWN_S:
        bb["flip"] = True
    return ("drive", l, r)


def fire_flipper(bb):
    """Ask main.py to send one flip, and keep pushing while the arm swings."""
    bb["flip"] = True
    return ("drive", ATTACK_SPEED, ATTACK_SPEED)


def build_tree():
    # Conditions test seen before the pose fields, which are missing when it's false
    return Selector(
        # out latches in track.py until r is pressed there; the laptop plays the sad song
        Sequence(Condition("out", lambda bb: bb.get("out")),
                 Action("Lost", lambda bb: ("drive", 0, 0))),
        # stale counts as idle, so the robot stops if the laptop goes quiet
        Sequence(Condition("not auto", lambda bb: bb.get("mode") != "auto" or not bb["fresh"]),
                 Action("Idle", lambda bb: ("drive", 0, 0))),
        # main.py only sets teleop while its last message is < 0.5 s old
        Sequence(Condition("teleop", lambda bb: "teleop" in bb),
                 Action("Teleop", teleop_drive)),
        Sequence(EdgeNear(),
                 Action("Edge escape", lambda bb: turn_and_drive(bb["center_bearing"], ESCAPE_SPEED))),
        # ultrasonic only, so it still works when vision drops out. -1 = no echo.
        # main.py stamps last_flip when it sends one; fires for a single tick, then cools down
        Sequence(Condition("contact", lambda bb: 0 <= bb.get("ultrasonic_cm", -1) < FLIP_CM),
                 Condition("flipper ready", lambda bb: bb["now"] - bb.get("last_flip", -1e9) >= FLIP_COOLDOWN_S),
                 Action("Flip", fire_flipper)),
        Sequence(Condition("opp in sight", lambda bb: bb.get("seen") and bb.get("opp_seen")),
                 Selector(
                     Sequence(Condition("lined up", lambda bb: abs(bb["opp_bearing"]) < ATTACK_DEG),
                              Condition("close", lambda bb: bb["opp_dist"] < ATTACK_CM),
                              Action("Attack", lambda bb: turn_and_drive(bb["opp_bearing"], ATTACK_SPEED))),
                     Action("Chase", lambda bb: turn_and_drive(bb["opp_bearing"], CHASE_SPEED)))),
        Action("Search", lambda bb: ("drive", SEARCH_SPEED, -SEARCH_SPEED)),
    )
