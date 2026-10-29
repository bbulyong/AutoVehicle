#!/usr/bin/env python3
"""Exactly one Profile-2 nominal CtrlCmd, preserving signal/stopline supervisor.

In OFF/DONE the original curvature controller is selected unchanged. In an
active highway mission the adaptive active-path controller is selected. The
strategy publishes a JSON state each tick *after* its path/stop/speed topics;
we use that atomic state to avoid desynchronizing the mode and safety decision.
"""
import json
import math
import threading
import time

import rospy
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import String


def brake_command():
    cmd = CtrlCmd()
    cmd.longlCmdType = 1
    cmd.steering = 0.0
    cmd.accel = 0.0
    cmd.brake = 1.0
    cmd.velocity = 0.0
    cmd.acceleration = 0.0
    return cmd


class HighwayMux:
    def __init__(self):
        self.lock = threading.RLock()
        self.timeout = float(rospy.get_param('~command_timeout_s', 0.35))
        self.status_timeout = float(rospy.get_param('~strategy_timeout_s', 0.65))
        self.boot_grace = float(rospy.get_param('~boot_grace_s', 4.0))
        self.start = time.monotonic()
        self.baseline = (None, None)
        self.highway = (None, None)
        self.status = (None, None)
        self.active_began = None
        self.last_mode = None
        self.pub = rospy.Publisher('/control/ctrl_cmd', CtrlCmd, queue_size=1)
        rospy.Subscriber('/control/profile2_baseline_cmd', CtrlCmd, self.baseline_cb, queue_size=1)
        rospy.Subscriber('/control/profile2_highway_cmd', CtrlCmd, self.highway_cb, queue_size=1)
        rospy.Subscriber('/highway_lane_strategy/state', String, self.status_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)

    def baseline_cb(self, msg):
        with self.lock:
            self.baseline = (msg, time.monotonic())

    def highway_cb(self, msg):
        with self.lock:
            self.highway = (msg, time.monotonic())

    def status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not all(k in data for k in ('state', 'active', 'stop')):
                return
            if type(data['active']) is not bool or type(data['stop']) is not bool:
                return
            with self.lock:
                old = self.status[0]
                if data['active'] and (old is None or not old.get('active')):
                    self.active_began = time.monotonic()
                self.status = (data, time.monotonic())
        except (ValueError, TypeError):
            rospy.logwarn_throttle(2.0, 'profile2 mux invalid highway status')

    def tick(self, _):
        now = time.monotonic()
        with self.lock:
            status, stamp = self.status
            fresh = stamp is not None and 0 <= now-stamp <= self.status_timeout
            if not fresh:
                # Startup tolerance is only for initialization; an active mission
                # MUST NOT revert silently to the original global path.
                if now-self.start < self.boot_grace and status is None:
                    selected, t = self.baseline
                    mode = 'baseline_startup'
                else:
                    selected, t = None, None
                    mode = 'strategy_stale_brake'
            elif status['stop']:
                selected, t = None, None
                mode = 'strategy_stop_brake'
            elif status['active']:
                selected, t = self.highway
                if self.active_began is not None and (t is None or t < self.active_began):
                    selected, t = None, None
                mode = 'highway'
            else:
                selected, t = self.baseline
                mode = 'baseline'
            if (selected is None or t is None or now-t > self.timeout
                or int(getattr(selected, 'longlCmdType', 1)) != 1
                or not all(math.isfinite(float(getattr(selected, k, 0.0)))
                           for k in ('steering', 'accel', 'brake'))):
                selected = brake_command()
                mode += '_brake'
            if mode != self.last_mode:
                rospy.logwarn('profile2 command mux: %s (highway state=%s)',
                              mode, None if status is None else status.get('state'))
                self.last_mode = mode
            self.pub.publish(selected)


if __name__ == '__main__':
    rospy.init_node('profile2_highway_mux')
    HighwayMux()
    rospy.spin()
