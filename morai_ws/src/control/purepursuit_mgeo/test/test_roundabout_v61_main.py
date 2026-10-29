import copy
import json
import math
from pathlib import Path
import unittest

import yaml

from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from purepursuit_mgeo.roundabout_mission.core import (
    Decision, Ego, Track, projected_radius,
)
from purepursuit_mgeo.roundabout_mission.integration import MissionStatusSource


WS = Path(__file__).resolve().parents[4]
CONFIG = WS / "config" / "roundabout_v61_main.yaml"
ROUTE = WS / "data" / "routes" / "2026_molit_comp_global_path.txt"
CIRCULATING = WS / "data" / "routes" / "roundabout_circulating_path.txt"


class MainRoundaboutConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.route = load_mgeo_path(str(ROUTE))
        cls.circulating = load_mgeo_path(str(CIRCULATING))

    def test_main_route_calibration_loads_v61_decision(self):
        decision = Decision(self.route, self.circulating, dict(self.cfg))
        self.assertEqual(decision.entry_decision_stop_index, 1792)
        self.assertEqual(len(self.route), 4430)
        self.assertEqual(len(self.circulating), 226)
        self.assertLess(decision.cs, decision.ce)
        self.assertEqual(self.cfg["version"], "6.1.0-main")
        self.assertTrue(self.cfg["lead_relative_following_enabled"])

    def test_circulating_conflict_matches_main_route(self):
        for route_index, circulating_index in ((1822, 42), (1832, 52)):
            route = self.route[route_index]
            circulating = self.circulating[circulating_index]
            self.assertLess(math.hypot(route.x-circulating.x,
                                       route.y-circulating.y), 1.0)

    def test_inactive_gate_does_not_stop_main_route(self):
        decision = Decision(self.route, self.circulating, dict(self.cfg))
        point = self.route[100]
        row = decision.step(Ego(point.x, point.y, 0.0, 2.0), 10.0,
                            fresh=True, obstacles_fresh=False)
        self.assertFalse(row["active"])
        self.assertFalse(row["stop_required"])
        self.assertEqual(row["decision_reason"], "csv_cruise")


class MainControllerMissionAdapterTest(unittest.TestCase):
    @staticmethod
    def status(stamp, active=False, target=2.0, stop=False, emergency=False):
        return json.dumps(dict(timestamp=stamp, active=active,
                               target_speed=target, stop_required=stop,
                               emergency=emergency, decision_reason="test"))

    def test_inactive_status_preserves_main_speed(self):
        source = MissionStatusSource(0.3)
        self.assertTrue(source.observe(self.status(10.0), 10.1, 20.0))
        self.assertEqual(source.apply(5.0, 10.2, 20.1),
                         (5.0, False, "csv_cruise"))

    def test_active_status_only_caps_main_speed(self):
        source = MissionStatusSource(0.3)
        self.assertTrue(source.observe(self.status(10.0, active=True, target=2.0),
                                       10.1, 20.0))
        self.assertEqual(source.apply(5.0, 10.2, 20.1), (2.0, False, "test"))
        self.assertEqual(source.apply(1.5, 10.2, 20.1), (1.5, False, "test"))

    def test_stop_and_stale_status_fail_safe_before_main_pi(self):
        source = MissionStatusSource(0.3)
        self.assertEqual(source.apply(5.0, 10.0, 20.0),
                         (0.0, True, "decision_stale"))
        self.assertTrue(source.observe(self.status(10.0, active=True, stop=True),
                                       10.1, 20.0))
        self.assertEqual(source.apply(5.0, 10.2, 20.1), (0.0, True, "test"))
        self.assertEqual(source.apply(5.0, 10.5, 20.4),
                         (0.0, True, "decision_stale"))

    def test_invalid_status_is_rejected(self):
        source = MissionStatusSource(0.3)
        self.assertFalse(source.observe("{}", 10.0, 20.0))
        self.assertFalse(source.observe(self.status(9.0), 10.0, 20.0))


class V61LeadFollowingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cfg.update(
            mission_indices=dict(approach=5, stop_line=20, entry=25,
                                 conflict_start=46, conflict_end=56,
                                 merge_complete=75, exit=100),
            circulating_conflict_indices=[145, 155],
            roi_map=[-10, 90, -160, 160],
            entry_decision_stop_index=20,
            trajectory_prediction_enabled=False,
            merge_speed=3.4,
        )
        cls.cfg = cfg
        cls.path = [PathPoint(0.5*i, 0.0, 0.0) for i in range(2001)]
        cls.circulating = [PathPoint(26.0, float(y), 0.0)
                           for y in range(-150, 151)]

    def engine(self):
        decision = Decision(self.path, self.circulating, copy.deepcopy(self.cfg))
        decision.state = "COMMIT"
        decision.entry_stop_completed = True
        decision.target = 3.4
        return decision

    def lead(self, gap, speed=2.0, track_id=10):
        radius = projected_radius(4.5, 1.8, 0.0, (1.0, 0.0))
        return Track(track_id, 14.0+self.cfg["ego_front_m"]+radius+gap,
                     0.0, 0.0, 4.5, 1.8, speed, 0.0)

    def step(self, ego_speed, gap, lead_speed=2.0, track_id=10):
        decision = self.engine()
        decision.update_tracks([self.lead(gap, lead_speed, track_id)], 1.0)
        return decision.step(Ego(14.0, 0.0, 0.0, ego_speed), 1.0, True, True)

    def test_closing_lead_uses_speed_cap_instead_of_predicted_stop(self):
        row = self.step(3.4, 1.3)
        self.assertTrue(row["lead_relative_following_active"])
        self.assertTrue(row["lead_prediction_stop_suppressed"])
        self.assertFalse(row["stop_required"])
        self.assertFalse(row["emergency"])
        self.assertGreater(row["target_speed"], 0.0)
        self.assertLessEqual(row["target_speed"], 2.0)

    def test_lead_id_change_does_not_change_control(self):
        rows = [self.step(3.4, 1.3, track_id=track_id)
                for track_id in (10, 999, 73)]
        values = [(row["target_speed"], row["stop_required"], row["emergency"])
                  for row in rows]
        self.assertEqual(values[0], values[1])
        self.assertEqual(values[1], values[2])

    def test_lead_pulling_away_does_not_stop(self):
        row = self.step(1.0, 1.2)
        self.assertFalse(row["stop_required"])
        self.assertFalse(row["emergency"])
        self.assertEqual(row["target_speed"], 2.0)


if __name__ == "__main__":
    unittest.main()
