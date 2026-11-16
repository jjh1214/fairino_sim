"""
Index conveyors: parts travel on a straight belt from an entry frame to a station frame.

Parts stop at the station. The belt stands still while a part waits there and runs again as
soon as the part is taken; parts follow each other at a fixed spacing. The IO
<conveyor>/part_ready is true while a part waits at the station. The model is deterministic, so
a check can run it in virtual time and predict where every part will be during a move or a
wait.
"""

from dataclasses import dataclass, field
import math

from fairino_workcell.cell import part_name, ready_io
import numpy as np

EPS = 1e-9


@dataclass
class Belt:
    """Runtime state of one conveyor."""

    items: list = field(default_factory=list)   # [part name, distance from the entry], lead first
    fed: int = 0                                 # parts put on the belt so far
    travel: float = math.inf                     # belt travel since the last part was put on

    def copy(self):
        return Belt([list(item) for item in self.items], self.fed, self.travel)

    def take(self, name):
        """Take a part off the belt; return True if it was on it."""
        for index, (item, _) in enumerate(self.items):
            if item == name:
                del self.items[index]
                return True
        return False


class ConveyorModel:
    """Path and rules of one index conveyor."""

    def __init__(self, name, spec, entry, station):
        """Make the model of the conveyor spec of object name; entry, station: 4x4 world poses."""
        self.name, self.part = name, spec.part
        self.speed, self.spacing, self.count = spec.speed, spec.spacing, spec.count
        self.origin = np.array(entry[:3, 3], dtype=float)
        offset = np.asarray(station[:3, 3], dtype=float) - self.origin
        self.length = float(np.linalg.norm(offset))
        self.direction = offset / self.length
        self.rotation = np.array(station[:3, :3], dtype=float)
        self.io = ready_io(name)

    def ready(self, belt):
        """Return True while a part waits at the station."""
        return bool(belt.items) and belt.items[0][1] >= self.length - EPS

    def can_feed(self, belt):
        return self.count == 0 or belt.fed < self.count

    def running(self, belt):
        return not self.ready(belt) and (bool(belt.items) or self.can_feed(belt))

    def feed_due(self, belt):
        return self.can_feed(belt) and belt.travel >= self.spacing - EPS

    def time_to_event(self, belt):
        """Return the time until a part arrives or is put on the belt (inf: standing still)."""
        if not self.running(belt):
            return math.inf
        times = []
        if belt.items:
            times.append((self.length - belt.items[0][1]) / self.speed)
        if self.can_feed(belt):
            times.append((self.spacing - belt.travel) / self.speed)
        return max(0.0, min(times))

    def run(self, belt, dt):
        """Move the belt for dt seconds; dt must not go past the next event."""
        distance = self.speed * dt
        for item in belt.items:
            item[1] = min(item[1] + distance, self.length)
        belt.travel += distance

    def pose(self, distance):
        """Return the 4x4 world pose of a part at a distance from the entry."""
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.origin + distance * self.direction
        return matrix


class Conveyors:
    """
    All conveyors of a cell, run together on a CellState.

    The state keeps the belts, the parts put in (named part_1, part_2, ... per part type), their
    poses and the sensor IO; the cell time advances with the conveyors.
    """

    def __init__(self, models):
        self.models = dict(models)                          # conveyor name -> ConveyorModel
        self.sensors = {model.io: model for model in self.models.values()}

    def start(self, state):
        """Give the state empty belts and put the first parts on them."""
        state.belts = {name: Belt() for name in self.models}
        self.advance(state, 0.0)

    def moving(self, state):
        """Return True if a conveyor is running."""
        return any(model.running(state.belts[name]) for name, model in self.models.items())

    def advance(self, state, dt):
        """Run the conveyors for dt seconds of cell time; return the names of new parts."""
        new = self._feed(state)
        remaining = max(0.0, float(dt))
        while remaining > 0.0:
            running = [(model, state.belts[name]) for name, model in self.models.items()
                       if model.running(state.belts[name])]
            if not running:
                break
            step = min([remaining] + [model.time_to_event(belt) for model, belt in running])
            step = max(step, min(remaining, 1e-9))     # always make progress
            for model, belt in running:
                model.run(belt, step)
            state.time += step
            remaining -= step
            new += self._feed(state)
        state.time += remaining
        self._update(state)
        return new

    def take(self, state, name):
        """Take a part off its belt (it was gripped); the conveyor may run on."""
        for belt in state.belts.values():
            belt.take(name)
        self._update(state)

    def next_event(self, state):
        """Return the time until a part arrives at a station or is put on a belt (inf: none)."""
        return min([model.time_to_event(state.belts[name])
                    for name, model in self.models.items()], default=math.inf)

    def time_until(self, state, io, value, limit):
        """
        Return (time, still) until a sensor has the value.

        time is None if it does not happen within limit; still tells that nothing moves any
        more, so it will never happen.
        """
        sim, elapsed = state.copy(), 0.0
        while sim.io.get(io, False) != value:
            step = self.next_event(sim)
            if step == math.inf:
                return None, True
            if elapsed + step > limit:
                return None, False
            self.advance(sim, step)
            elapsed += step
        return elapsed, False

    def time_to_still(self, state, limit):
        """Return the time until no conveyor runs (at most limit)."""
        sim, elapsed = state.copy(), 0.0
        while elapsed < limit:
            step = self.next_event(sim)
            if step == math.inf:
                return elapsed
            step = min(step, limit - elapsed)
            self.advance(sim, step)
            elapsed += step
        return limit

    def predict(self, state, times):
        """Return the poses of the parts on belts at each of the times (rising, from now)."""
        sim, now, frames = state.copy(), 0.0, []
        for t in times:
            self.advance(sim, t - now)
            now = t
            frames.append({name: sim.poses[name]
                           for belt in sim.belts.values() for name, _ in belt.items})
        return frames, sim.parts

    def _feed(self, state):
        new = []
        for name, model in self.models.items():
            belt = state.belts[name]
            if model.feed_due(belt):
                number = state.counters.get(model.part, 0) + 1
                state.counters[model.part] = number
                part = part_name(model.part, number)
                state.parts[part] = model.part
                belt.items.append([part, 0.0])
                belt.fed += 1
                belt.travel = 0.0
                state.poses[part] = model.pose(0.0)
                new.append(part)
        return new

    def _update(self, state):
        for name, model in self.models.items():
            belt = state.belts[name]
            for part, distance in belt.items:
                state.poses[part] = model.pose(distance)
            state.io[model.io] = model.ready(belt)
