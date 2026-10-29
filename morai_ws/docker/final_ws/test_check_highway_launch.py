#!/usr/bin/env python3
import unittest

import check_highway_launch as check


class CurvatureLaunchValidationTest(unittest.TestCase):
    def base_names(self):
        return sorted(check.CURVATURE_REQUIRED)

    def test_roundabout_nodes_are_required_and_are_the_only_lidar_nodes(self):
        names = self.base_names() + sorted(check.ROUNDABOUT_REQUIRED)
        check.validate_curvature_names(names, '\n'.join(names), roundabout_enabled=True)

    def test_unexpected_lidar_node_is_rejected(self):
        names = self.base_names() + sorted(check.ROUNDABOUT_REQUIRED) + ['/legacy_lidar_avoidance']
        with self.assertRaises(AssertionError):
            check.validate_curvature_names(names, '\n'.join(names), roundabout_enabled=True)

    def test_missing_roundabout_node_is_rejected(self):
        names = self.base_names() + sorted(check.ROUNDABOUT_REQUIRED - {'/roundabout_merge_gate'})
        with self.assertRaises(AssertionError):
            check.validate_curvature_names(names, '\n'.join(names), roundabout_enabled=True)

    def test_roundabout_nodes_are_absent_when_disabled(self):
        names = self.base_names()
        check.validate_curvature_names(names, '\n'.join(names), roundabout_enabled=False)

    def test_roundabout_node_left_enabled_is_rejected(self):
        names = self.base_names() + ['/roundabout_merge_gate']
        with self.assertRaises(AssertionError):
            check.validate_curvature_names(names, '\n'.join(names), roundabout_enabled=False)


if __name__ == '__main__':
    unittest.main()
