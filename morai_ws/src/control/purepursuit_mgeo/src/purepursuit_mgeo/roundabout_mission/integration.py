"""Contract for the main controller: compose BEFORE its pedal PI.

Use the timestamped JSON status, with both ROS-time and wall-time watchdogs.
An inactive gate adds no speed cap; active gate caps the existing planner.
Never use allowed=False as a global stop (it is false during normal cruise).
"""
import json
import math


def apply_mission(base_target, status, status_fresh):
    if not status_fresh or status is None:
        return 0., True, 'decision_stale'
    if status['stop_required'] or status.get('emergency', False):
        return 0., True, status['decision_reason']
    if status.get('active', False):
        return min(base_target, status['target_speed']), False, status['decision_reason']
    return base_target, False, 'csv_cruise'


class MissionStatusSource:
    """Validate and age the atomic gate status used by new_final_ws.

    The decision timestamp and callback receipt time are both checked.  A
    missing or stale gate therefore requests a stop before the main pedal PI,
    while an inactive, fresh gate leaves the route planner unchanged.
    """

    def __init__(self, timeout_s=0.3):
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError('roundabout decision timeout must be positive')
        self.timeout_s = timeout_s
        self._sample = None

    @staticmethod
    def _validated(payload):
        row = json.loads(payload)
        if not isinstance(row, dict):
            raise ValueError('roundabout status must be a JSON object')
        for key in ('timestamp', 'target_speed'):
            value = row.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('invalid roundabout status '+key)
        if row['timestamp'] <= 0.0 or row['target_speed'] < 0.0:
            raise ValueError('negative/zero roundabout time or speed')
        for key in ('active', 'stop_required'):
            if type(row.get(key)) is not bool:
                raise ValueError('invalid roundabout status '+key)
        if 'emergency' in row and type(row['emergency']) is not bool:
            raise ValueError('invalid roundabout emergency flag')
        if not isinstance(row.get('decision_reason'), str):
            raise ValueError('missing roundabout decision reason')
        return row

    def observe(self, payload, ros_now, wall_now):
        values = (float(ros_now), float(wall_now))
        if not all(math.isfinite(value) for value in values):
            return False
        try:
            row = self._validated(payload)
        except (TypeError, ValueError, json.JSONDecodeError):
            return False
        stamp = float(row['timestamp'])
        if not -0.05 <= values[0] - stamp <= self.timeout_s:
            return False
        previous = self._sample
        if previous is not None and values[0] < previous[0]['timestamp'] - 0.05:
            # A simulator reset starts a new source-time epoch.
            previous = None
            self._sample = None
        if previous is not None and stamp <= previous[0]['timestamp']:
            return False
        self._sample = (row, values[1])
        return True

    def apply(self, base_target, ros_now, wall_now):
        base_target = float(base_target)
        if not math.isfinite(base_target) or base_target < 0.0:
            raise ValueError('base target speed must be finite and nonnegative')
        sample = self._sample
        if sample is None:
            return apply_mission(base_target, None, False)
        row, received = sample
        fresh = (
            -0.05 <= float(ros_now) - row['timestamp'] <= self.timeout_s
            and 0.0 <= float(wall_now) - received <= self.timeout_s
        )
        return apply_mission(base_target, row, fresh)
