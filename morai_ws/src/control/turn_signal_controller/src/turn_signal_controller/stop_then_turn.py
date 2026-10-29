"""Per-junction stop permission, measured with vehicle speed and route distance."""
import math


def stop_then_turn(event, speed, gap, now, valid, dwell=1.0):
    """Release only after a continuous stop at the configured bumper target.

    State belongs to this event, so permission cannot reach the next junction.
    Caller retains pose, obstacle, speed freshness and command checks.
    """
    if not valid or any(not isinstance(v, (int, float)) or not math.isfinite(v)
                        for v in (speed, gap, now)):
        event.pop('right_stop_since', None)
        event.pop('right_stop_tick', None)
        return False
    if event.get('right_stop_released'):
        return True
    previous = event.get('right_stop_tick')
    event['right_stop_tick'] = now
    if (speed < 0 or speed > 0.15 or not -0.2 <= gap <= 0.75
            or previous is None or not 0 < now - previous <= 0.25):
        event.pop('right_stop_since', None)
        return False
    started = event.setdefault('right_stop_since', now)
    if now - started >= dwell:
        event['right_stop_released'] = True
        return True
    return False
