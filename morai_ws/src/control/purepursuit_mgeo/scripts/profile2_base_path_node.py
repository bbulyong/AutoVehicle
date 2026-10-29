#!/usr/bin/env python3
"""Profile 2 global-route feeder for the supplied V9 highway strategy.

Does NOT implement obstacle avoidance. The old Profile-2 global-route and
signal/stopline controller remain authoritative outside highway mode.
"""
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from purepursuit_mgeo.path import load_mgeo_path


class BaseRouteFeed:
    def __init__(self):
        self.points = load_mgeo_path(rospy.get_param('~path_file'))
        self.frame = rospy.get_param('~map_frame', 'map')
        self.max_pose_age_s = float(rospy.get_param('~odom_timeout_s', 0.5))
        self.forward_m = float(rospy.get_param('~forward_m', 90.0))
        self.back_m = float(rospy.get_param('~back_m', 4.0))
        self.lock = threading.RLock()
        self.pose = None
        self.last_odom = None
        self.last_idx = None
        self.path_pub = rospy.Publisher('~active_path', Path, queue_size=1)
        self.stop_pub = rospy.Publisher('~stop_required', Bool, queue_size=1)
        rospy.Subscriber('/localization/odometry', Odometry, self.on_odom, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def on_odom(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame:
            return
        p = msg.pose.pose.position
        if not math.isfinite(p.x) or not math.isfinite(p.y):
            return
        with self.lock:
            self.pose = (p.x, p.y)
            self.last_odom = rospy.Time.now()

    def nearest(self, x, y):
        n = len(self.points)
        if self.last_idx is None:
            candidates = range(n)
        else:
            candidates = range(max(0, self.last_idx-35), min(n, self.last_idx+180))
        i = min(candidates, key=lambda k: (self.points[k].x-x)**2+(self.points[k].y-y)**2)
        # Do not snap across distant disconnected route segments on a GPS jump.
        if (self.points[i].x-x)**2 + (self.points[i].y-y)**2 > 20.0**2:
            i = min(range(n), key=lambda k: (self.points[k].x-x)**2+(self.points[k].y-y)**2)
        self.last_idx = i
        return i

    def tick(self, _):
        with self.lock:
            now = rospy.Time.now()
            if self.last_odom is None or (now-self.last_odom).to_sec() > self.max_pose_age_s:
                self.stop_pub.publish(Bool(data=True))
                return
            x, y = self.pose
            idx = self.nearest(x, y)
            start = idx
            d = 0.0
            while start > 0 and d < self.back_m:
                a, b = self.points[start], self.points[start-1]
                d += math.hypot(a.x-b.x, a.y-b.y)
                start -= 1
            end = max(idx+1, start+1)
            d = 0.0
            while end < len(self.points)-1 and d < self.forward_m:
                a, b = self.points[end-1], self.points[end]
                d += math.hypot(a.x-b.x, a.y-b.y)
                end += 1
            msg = Path()
            msg.header.stamp = now
            msg.header.frame_id = self.frame
            for p in self.points[start:end+1]:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x, p.y, p.z
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
            if len(msg.poses) < 2:
                self.stop_pub.publish(Bool(data=True))
                return
            self.path_pub.publish(msg)
            self.stop_pub.publish(Bool(data=False))


if __name__ == '__main__':
    rospy.init_node('profile2_base_path')
    BaseRouteFeed()
    rospy.spin()
