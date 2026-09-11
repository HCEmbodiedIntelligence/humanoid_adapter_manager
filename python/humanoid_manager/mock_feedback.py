"""One-control-frame feedback delay; no hardware, dynamics or command publishing."""
import math
import threading


ARM_NAMES = tuple(f'openarmx_{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8))
GRIPPER_NAMES = ('left_gripper', 'right_gripper')


class DelayedFeedback:
    def __init__(self, names, initial):
        self.names = tuple(names)
        self._index = {name: i for i, name in enumerate(self.names)}
        self._target = list(initial)
        self._staged = list(initial)
        self._lock = threading.Lock()

    def accept(self, names, positions, velocity=(), effort=()):
        """Validate the whole command before atomically accepting named partial updates."""
        if not names or len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('command needs unique joint names and matching positions')
        if any(name not in self._index for name in names):
            raise ValueError('command contains an unknown joint name')
        for values in (positions, velocity, effort):
            if values and (len(values) != len(names) or not all(math.isfinite(x) for x in values)):
                raise ValueError('command arrays must match names and contain finite numbers')
        with self._lock:
            for name, value in zip(names, positions):
                self._target[self._index[name]] = float(value)

    def tick(self):
        """Publish the staged frame; stage the latest target for the next tick."""
        with self._lock:
            output = self._staged[:]
            self._staged = self._target[:]
        return output
