#!/usr/bin/env python3
"""Allow V9 highway activation only with a fresh, inactive V6.1 roundabout.

This is a mission interlock, not a new steering controller. The user-supplied
V9 highway planner remains byte-for-byte unchanged.
"""
import json
import threading
import time

import rospy
from std_msgs.msg import Bool, String


class HighwayPermission:
    def __init__(self):
        self.lock = threading.RLock()
        self.roundabout = None
        self.roundabout_at = None
        self.camera = False
        self.camera_at = None
        self.request = False
        self.request_at = None
        self.roundabout_timeout = float(rospy.get_param('~roundabout_timeout_s', 0.75))
        self.camera_timeout = float(rospy.get_param('~camera_timeout_s', 0.75))
        self.request_timeout = float(rospy.get_param('~request_timeout_s', 0.75))
        self.environment_pub = rospy.Publisher('/profile2_v9/highway_environment', Bool, queue_size=1)
        self.request_pub = rospy.Publisher('/profile2_v9/highway_request', Bool, queue_size=1)
        rospy.Subscriber('/roundabout_merge_gate/status', String, self.on_roundabout, queue_size=1)
        rospy.Subscriber('/perception/camera/highway_environment', Bool, self.on_camera, queue_size=1)
        rospy.Subscriber('/planning/highway_lane_change_request', Bool, self.on_request, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)

    def on_roundabout(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict) or type(state.get('active')) is not bool:
                raise ValueError('missing roundabout active flag')
            with self.lock:
                self.roundabout = state['active']
                self.roundabout_at = time.monotonic()
        except (ValueError, TypeError):
            rospy.logwarn_throttle(2.0, 'V9 highway gate: invalid roundabout status')

    def on_camera(self, msg):
        with self.lock:
            self.camera = bool(msg.data)
            self.camera_at = time.monotonic()

    def on_request(self, msg):
        with self.lock:
            self.request = bool(msg.data)
            self.request_at = time.monotonic()

    @staticmethod
    def fresh(stamp, timeout, now):
        return stamp is not None and 0.0 <= now-stamp <= timeout

    def tick(self, _):
        with self.lock:
            now = time.monotonic()
            roundabout_clear = (self.fresh(self.roundabout_at, self.roundabout_timeout, now)
                                and self.roundabout is False)
            camera_ok = (roundabout_clear and self.camera
                         and self.fresh(self.camera_at, self.camera_timeout, now))
            request_ok = (roundabout_clear and self.request
                          and self.fresh(self.request_at, self.request_timeout, now))
            self.environment_pub.publish(Bool(data=camera_ok))
            self.request_pub.publish(Bool(data=request_ok))


if __name__ == '__main__':
    rospy.init_node('profile2_v9_highway_permission')
    HighwayPermission()
    rospy.spin()
