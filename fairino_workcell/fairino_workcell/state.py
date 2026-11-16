"""State of a work cell during a simulation or a check session (plain data)."""

from dataclasses import dataclass, field


@dataclass
class CellState:
    """Robot, objects, IO and conveyors at one moment of the cell time."""

    joints: tuple
    poses: dict                                      # movable object -> 4x4 world pose (not held)
    held: dict = field(default_factory=dict)         # object -> 4x4 pose relative to the TCP
    next_allowed: set = field(default_factory=set)   # contacts allowed in the next move only
    time: float = 0.0                                # cell time [s]
    io: dict = field(default_factory=dict)           # virtual IO
    belts: dict = field(default_factory=dict)        # conveyor -> conveyor.Belt
    parts: dict = field(default_factory=dict)        # part put in by a conveyor -> part type
    counters: dict = field(default_factory=dict)     # part type -> parts put in so far

    def copy(self):
        return CellState(tuple(self.joints), {k: v.copy() for k, v in self.poses.items()},
                         {k: v.copy() for k, v in self.held.items()}, set(self.next_allowed),
                         self.time, dict(self.io), {k: b.copy() for k, b in self.belts.items()},
                         dict(self.parts), dict(self.counters))
