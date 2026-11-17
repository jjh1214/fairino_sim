"""
Conveyors: parts travel on a straight belt, put on it at an entry frame at a fixed spacing.

Index conveyors bring the parts to a station frame where they stop. The belt stands still while
a part waits there and runs again as soon as the part is taken. The IO <conveyor>/part_ready is
true while a part waits at the station.

Continuous conveyors keep running. The station frame is a sensor: <conveyor>/part_ready is true
while a part that has passed it is still on the belt, to be picked on the move. Parts that
reach the exit frame leave the cell; they are recorded as missed.

Parts lie on the belt centre line, turned like the station frame, unless the conveyor has a
placement error: then each part lies a random distance across and along the belt and is turned
about the vertical. Along the belt, an index conveyor still stops the part at the station (its
sensor stops the belt); the part only arrives a little earlier or later.

The model is deterministic, so a check can run it in virtual time and predict where every part
will be during a move or a wait. Placement errors are drawn from the seed and the part number.
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
    missed: list = field(default_factory=list)   # parts that left at the exit (continuous)
    offsets: dict = field(default_factory=dict)  # part -> (m across the belt, rad about vertical)

    def copy(self):
        return Belt([list(item) for item in self.items], self.fed, self.travel,
                    list(self.missed), dict(self.offsets))

    def take(self, name):
        """Take a part off the belt; return True if it was on it."""
        for index, (item, _) in enumerate(self.items):
            if item == name:
                del self.items[index]
                self.offsets.pop(name, None)
                return True
        return False


class ConveyorModel:
    """Path and rules of one conveyor."""

    def __init__(self, name, spec, entry, station, exit_pose=None):
        """Make the model of the conveyor spec of object name; frames as 4x4 world poses."""
        self.name, self.part, self.mode = name, spec.part, spec.mode
        self.speed, self.spacing, self.count = spec.speed, spec.spacing, spec.count
        self.continuous = spec.mode == 'continuous'
        self.origin = np.array(entry[:3, 3], dtype=float)
        end = exit_pose if self.continuous else station
        offset = np.asarray(end[:3, 3], dtype=float) - self.origin
        self.length = float(np.linalg.norm(offset))       # to the station (index) or the exit
        self.direction = offset / self.length
        self.velocity = self.speed * self.direction
        self.sensor = (float(np.dot(np.asarray(station[:3, 3]) - self.origin, self.direction))
                       if self.continuous else self.length)
        self.rotation = np.array(station[:3, :3], dtype=float)
        up = self.rotation[:, 2]                            # the station frame's z: vertical
        across = np.cross(up, self.direction)
        self.across = across / max(np.linalg.norm(across), EPS)
        self.error = spec.placement_error
        self.io = ready_io(name)

    def ready(self, belt):
        """Return True while a part waits at the station, or has passed the sensor."""
        if self.continuous:
            return any(distance >= self.sensor - EPS for _, distance in belt.items)
        return bool(belt.items) and belt.items[0][1] >= self.length - EPS

    def ready_part(self, belt):
        """Return the part to pick: at the station, or the first one past the sensor."""
        if self.ready(belt):
            return belt.items[0][0]
        return None

    def can_feed(self, belt):
        return self.count == 0 or belt.fed < self.count

    def running(self, belt):
        if self.continuous:
            return bool(belt.items) or self.can_feed(belt)
        return not self.ready(belt) and (bool(belt.items) or self.can_feed(belt))

    def feed_due(self, belt):
        return self.can_feed(belt) and belt.travel >= self.spacing - EPS

    def time_to_event(self, belt):
        """Return the time until a part arrives, passes the sensor, leaves or is put on."""
        if not self.running(belt):
            return math.inf
        times = []
        if belt.items:
            times.append((self.length - belt.items[0][1]) / self.speed)
        if self.continuous:
            times += [(self.sensor - distance) / self.speed for _, distance in belt.items
                      if distance < self.sensor - EPS]
        if self.can_feed(belt):
            times.append((self.spacing - belt.travel) / self.speed)
        return max(0.0, min(times))

    def run(self, belt, dt):
        """Move the belt for dt seconds; dt must not go past the next event."""
        distance = self.speed * dt
        for item in belt.items:
            item[1] = min(item[1] + distance, self.length)
        belt.travel += distance

    def leave(self, belt):
        """Take the parts that reached the exit off a continuous belt; return their names."""
        if not self.continuous:
            return []
        gone = [name for name, distance in belt.items if distance >= self.length - EPS]
        belt.items = [item for item in belt.items if item[0] not in gone]
        belt.missed.extend(gone)
        for name in gone:
            belt.offsets.pop(name, None)
        return gone

    def placement(self, number):
        """
        Return (along, across, yaw) of the n-th part of the cell: where it lies on the belt.

        along and across are metres from its nominal place, yaw radians about the vertical;
        all zero without a placement error.
        """
        if self.error is None:
            return 0.0, 0.0, 0.0
        draw = np.random.default_rng([self.error.seed, number]).uniform(-1.0, 1.0, 3)
        return (float(draw[0] * self.error.along), float(draw[1] * self.error.across),
                float(draw[2] * self.error.yaw))

    def pose(self, distance, offset=None):
        """Return the 4x4 world pose of a part at a distance from the entry."""
        across, yaw = offset if offset is not None else (0.0, 0.0)
        matrix = np.eye(4)
        c, s = math.cos(yaw), math.sin(yaw)
        matrix[:3, :3] = self.rotation @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        matrix[:3, 3] = self.origin + distance * self.direction + across * self.across
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
            for model, belt in running:
                for part in model.leave(belt):
                    state.poses.pop(part, None)
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
                along, across, yaw = model.placement(number)
                belt.items.append([part, along])
                belt.offsets[part] = (across, yaw)
                belt.fed += 1
                belt.travel = 0.0
                state.poses[part] = model.pose(along, belt.offsets[part])
                new.append(part)
        return new

    def _update(self, state):
        for name, model in self.models.items():
            belt = state.belts[name]
            for part in model.leave(belt):
                state.poses.pop(part, None)
            for part, distance in belt.items:
                state.poses[part] = model.pose(distance, belt.offsets.get(part))
            state.io[model.io] = model.ready(belt)

    def missed(self, state):
        """Return the parts that left continuous conveyors without being picked."""
        return [part for belt in state.belts.values() for part in belt.missed]
