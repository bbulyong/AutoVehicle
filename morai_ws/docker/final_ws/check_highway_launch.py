#!/usr/bin/env python3
"""Resolve complete launch includes with real ROS; no sensors/control started."""
import os
from pathlib import Path
import subprocess


CURVATURE_REQUIRED = {
    '/curvature_speed_purepursuit',
    '/curvature_signal_controller',
    '/highway_lane_camera',
    '/lane_info_contract',
    '/yolo_camera',
    '/traffic_light_stop',
    '/camera_debug_dashboard',
}
CURVATURE_EXCLUDED = {
    '/control_mux',
    '/final_ws_stopline_controller',
    '/avoidance_path_manager',
    '/bypass_lane_guard',
    '/avoidance_frenet_debug',
    '/roi_sensor_safety_adapter',
    '/intersection_environment',
    '/pedestrian_crossing_fusion',
}
ROUNDABOUT_REQUIRED = {
    '/morai_lidar_udp_source',
    '/roundabout_lidar_tracking',
    '/roundabout_merge_gate',
}
ROUNDABOUT_LIDAR_NODES = {
    '/morai_lidar_udp_source',
    '/roundabout_lidar_tracking',
}


def validate_curvature_names(names, output, roundabout_enabled):
    assert len(names) == len(set(names)), output
    for required in CURVATURE_REQUIRED:
        assert required in names, (required, output)
    for excluded in CURVATURE_EXCLUDED:
        assert excluded not in names, (excluded, output)

    if roundabout_enabled:
        for required in ('/profile2_base_path','/profile2_v9_highway_permission',
                         '/highway_lane_strategy','/profile2_v9_highway_controller',
                         '/profile2_v9_roundabout_mux',
                         '/profile2_v9_obstacle_states','/profile2_v9_merge_gap'):
            assert required in names, (required, output)
    else:
        for excluded in ('/highway_lane_strategy', '/profile2_v9_roundabout_mux',
                         '/profile2_v9_merge_gap'):
            assert excluded not in names, (excluded, output)

    present_roundabout = ROUNDABOUT_REQUIRED.intersection(names)
    if roundabout_enabled:
        assert present_roundabout == ROUNDABOUT_REQUIRED, (
            'roundabout nodes missing', ROUNDABOUT_REQUIRED - present_roundabout, output)
        expected_lidar = ROUNDABOUT_LIDAR_NODES
    else:
        assert not present_roundabout, ('roundabout nodes remained disabled', present_roundabout, output)
        expected_lidar = set()

    present_lidar = {name for name in names if 'lidar' in name.lower()}
    assert present_lidar == expected_lidar, (
        'unexpected curvature-signal LiDAR nodes', present_lidar - expected_lidar,
        'missing required LiDAR nodes', expected_lidar - present_lidar, output)


def main():
    ws=Path(__file__).resolve().parents[2]
    env=dict(os.environ, MORAI_WS=str(ws))
    for control,yolo in (('false','true'),('true','true'),('false','false')):
        output=subprocess.check_output(['roslaunch','--nodes','morai_bringup',
            'final_ws_highway_bringup.launch','workspace_path:='+str(ws),
            'enable_control:='+control,'enable_yolo:='+yolo],env=env,text=True)
        names=output.splitlines()
        assert len(names)==len(set(names)), output
        for required in ('/adaptive_curvature_purepursuit','/control_mux',
                         '/avoidance_path_manager','/highway_lane_strategy'):
            assert required in names, (required,output)
        # The highway launch uses the ROI camera stack, whose lane runner is
        # named lane_camera (sometimes under an inherited namespace). The
        # dedicated highway_lane_camera node belongs to curvature_signal.
        assert any(name.endswith('/lane_camera') for name in names), ('ROI lane camera missing',output)
        assert ('/yolo_camera' in names) == (yolo == 'true'), (
            'ROI YOLO camera configuration mismatch', yolo, output)
        assert '/purepursuit_mgeo' not in names and '/curvature_speed_purepursuit' not in names
    print('HIGHWAY_LAUNCH_OK: ROS resolved all includes; control was not started')
    for control in ('false', 'true'):
        output=subprocess.check_output(['roslaunch','--nodes','morai_bringup',
            'final_ws_curvature_signal.launch','workspace_path:='+str(ws),
            'enable_control:='+control],env=env,text=True)
        names=output.splitlines()
        validate_curvature_names(names, output, roundabout_enabled=True)

    output=subprocess.check_output(['roslaunch','--nodes','morai_bringup',
        'final_ws_curvature_signal.launch','workspace_path:='+str(ws),
        'enable_control:=false','enable_roundabout_mission:=false',
        'enable_highway_v9:=false'],env=env,text=True)
    validate_curvature_names(output.splitlines(), output, roundabout_enabled=False)
    print('CURVATURE_SIGNAL_LAUNCH_OK: ROS resolved monitor/drive; control was not started')


if __name__=='__main__': main()

