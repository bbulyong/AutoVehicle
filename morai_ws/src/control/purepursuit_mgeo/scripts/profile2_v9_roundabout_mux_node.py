#!/usr/bin/env python3
"""Select the V6.1 baseline except during an explicitly engaged V9 highway mode.

The original V6.1 curvature controller retains control of roundabouts,
stops and traffic lights. This node alone publishes /control/ctrl_cmd; the
original maneuver_fusion node remains the sole /ctrl_cmd publisher.
"""
import json
import math
import threading
import time

import rospy
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import String


class HighwayRoundaboutMux:
    HIGHWAY_STATES = frozenset(('WAIT_GAP', 'LANE_CHANGE', 'INNER_HOLD', 'REJOIN'))
    COMMITTED_STATES = frozenset(('LANE_CHANGE', 'INNER_HOLD', 'REJOIN'))

    def __init__(self):
        self.lock = threading.RLock()
        self.command_timeout = float(rospy.get_param('~command_timeout_s', 0.4))
        self.strategy_timeout = float(rospy.get_param('~strategy_timeout_s', 0.65))
        self.roundabout_timeout = float(rospy.get_param('~roundabout_timeout_s', 0.75))
        self.boot_grace_s = float(rospy.get_param('~boot_grace_s', 4.0))
        self.start = time.monotonic()
        self.baseline = (None, None)
        self.highway = (None, None)
        self.strategy = (None, None)
        self.roundabout = (None, None)
        self.engaged = False
        self.engaged_at = None
        self.last_mode = None
        self.publisher = rospy.Publisher('/control/ctrl_cmd', CtrlCmd, queue_size=1)
        rospy.Subscriber('/control/profile2_baseline_cmd', CtrlCmd, self.on_baseline, queue_size=1)
        rospy.Subscriber('/control/profile2_highway_cmd', CtrlCmd, self.on_highway, queue_size=1)
        rospy.Subscriber('/highway_lane_strategy/state', String, self.on_strategy, queue_size=1)
        rospy.Subscriber('/roundabout_merge_gate/status', String, self.on_roundabout, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)

    def on_baseline(self, msg):
        with self.lock:
            self.baseline = (msg, time.monotonic())

    def on_highway(self, msg):
        with self.lock:
            self.highway = (msg, time.monotonic())

    def on_strategy(self, msg):
        try:
            value = json.loads(msg.data)
            if (not isinstance(value, dict) or
                    value.get('state') not in self.HIGHWAY_STATES | {'OFF', 'DONE'} or
                    type(value.get('active')) is not bool or
                    type(value.get('stop')) is not bool):
                raise ValueError('invalid strategy state contract')
            with self.lock:
                self.strategy = (value, time.monotonic())
        except (ValueError, TypeError):
            rospy.logwarn_throttle(2.0, 'V9 mux: invalid strategy state')

    def on_roundabout(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or type(value.get('active')) is not bool:
                raise ValueError('missing roundabout active flag')
            with self.lock:
                self.roundabout = (value, time.monotonic())
        except (ValueError, TypeError):
            rospy.logwarn_throttle(2.0, 'V9 mux: invalid roundabout state')

    @staticmethod
    def fresh(stamp, timeout, now):
        return stamp is not None and 0 <= now-stamp <= timeout

    @staticmethod
    def full_brake():
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.steering = 0.0
        cmd.accel = 0.0
        cmd.brake = 1.0
        cmd.velocity = 0.0
        cmd.acceleration = 0.0
        return cmd

    def tick(self, _):
        now = time.monotonic()
        with self.lock:
            s, s_at = self.strategy
            r, r_at = self.roundabout
            strategy_fresh = self.fresh(s_at, self.strategy_timeout, now)
            roundabout_fresh = self.fresh(r_at, self.roundabout_timeout, now)
            roundabout_active = bool(r['active']) if roundabout_fresh else None
            highway_requested = bool(
                strategy_fresh and s['active'] and s['state'] in self.HIGHWAY_STATES
            )
            if highway_requested and roundabout_active is False and not self.engaged:
                self.engaged = True
                self.engaged_at = now
            elif strategy_fresh and s['state'] in ('OFF', 'DONE') and not s['active']:
                self.engaged = False
                self.engaged_at = None

            mode = 'baseline'
            source = self.baseline
            if self.engaged:
                # Never silently revert to global-route steering mid-change.
                if (not strategy_fresh or s['stop'] or not s['active'] or
                        roundabout_active is not False):
                    source = (None, None)
                    mode = 'highway_safety_stop'
                else:
                    source = self.highway
                    mode = 'highway'
                    if self.engaged_at is not None and (
                        source[1] is None or source[1] < self.engaged_at):
                        source = (None, None)
            elif highway_requested:
                # Roundabout has priority; WAIT_GAP cannot override V6.1.
                if s['state'] in self.COMMITTED_STATES:
                    source = (None, None)
                    mode = 'mission_conflict_stop'
                else:
                    mode = 'roundabout_priority'
            elif not strategy_fresh and now-self.start > self.boot_grace_s:
                # Losing the strategy before it engages does not disable V6.1.
                mode = 'baseline_strategy_unavailable'

            cmd, at = source
            if (cmd is None or not self.fresh(at, self.command_timeout, now)
                    or int(getattr(cmd, 'longlCmdType', -1)) != 1
                    or not all(math.isfinite(float(getattr(cmd, key, 0.0)))
                               for key in ('steering', 'accel', 'brake'))
                    or not (0.0 <= float(cmd.accel) <= 1.0 and
                            0.0 <= float(cmd.brake) <= 1.0)):
                cmd = self.full_brake()
                mode += '_brake'
            if mode != self.last_mode:
                rospy.logwarn('Profile2 V6.1/V9 mux: %s roundabout_active=%s highway=%s',
                              mode, roundabout_active, s.get('state') if s else '?')
                self.last_mode = mode
            self.publisher.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('profile2_highway_mux')
    HighwayRoundaboutMux()
    rospy.spin()
