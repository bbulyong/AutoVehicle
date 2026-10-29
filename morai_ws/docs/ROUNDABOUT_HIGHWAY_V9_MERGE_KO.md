# Profile 2: V6.1 Roundabout + preserved Highway V9

This experimental integration starts from upstream `test/roundabout` commit
`5944c565727a963d66754355f4debf3b443e58c0`. The source files for the
existing roundabout V6.1 mission, curvature controller, signal/stopline fusion
and Competition Vehicle Status parser are not changed.

Highway V9 source: `highway_lane_strategy_v9_node.py`. It is byte-for-byte
the uploaded 2,220-line V9 (Git blob `e9d88255f41c4a4dca603cec872361708a38b5dd`).

## Command ownership

- Baseline: existing V6.1 `curvature_speed_purepursuit_node.py`, now writes
  `/control/profile2_baseline_cmd` when `enable_highway_v9:=true`.
- Highway: `adaptive_curvature_purepursuit_node.py` follows V9
  `/highway_lane_strategy/active_path` and writes `/control/profile2_highway_cmd`.
- Single selector: `profile2_v9_roundabout_mux_node.py` writes
  `/control/ctrl_cmd`, then the **unmodified** signal/stopline supervisor
  writes `/ctrl_cmd` for the UDP bridge.
- Existing V6.1 roundabout gate has priority. Stale gate during a committed
  highway change causes a braking command, never silent reversion to global path.
- One original V6.1 LiDAR UDP source/tracker; two extra consumers derive
  tracked-obstacle states and adjacent-lane merge-gap detection. No second
  UDP socket on LiDAR port 2001.
- Highway permission requires a fresh V6.1 roundabout status with
  `active=false`; camera highway detection or a route request also required.
- The camera/environment and request are mirrored into a gated, V9-only topic.
  When V6.1 is active or stale, a new V9 lane change is not authorized.

## Host example without modifying previous installation

```bash
cd ~
git clone --single-branch --branch feature/roundabout-v61-highway-v9-integration \
  https://github.com/bbulyong/AutoVehicle.git AutoVehicle_roundabout_V9
cd ~/AutoVehicle_roundabout_V9/morai_ws/docker/final_ws
cp highway.env.example highway.env
cp highway-test.env.example highway-test.env
```

Edit ONLY the copied files: `UBUNTU_IP=192.168.10.21`,
`MORAI_IP=192.168.10.14`, `IMAGE_NAME=morai-rbv9:cpu`,
`CONTAINER_NAME=morai-rbv9-net10`, `TORCH_FLAVOR=cpu`.
Create a new Docker image and container; do not reuse or overwrite old tags.
Run `bash run_highway.sh build` and examine build errors. Then
`bash run_highway.sh start`; stop it before patching with
`bash install_curvature_signal.sh --config`; start it again.
`bash run_test.sh monitor 2` **does not send UDP controls**.

## Diagnostics

```bash
rostopic info /control/ctrl_cmd
rostopic info /ctrl_cmd
rostopic echo -n 1 /roundabout_merge_gate/status
rostopic echo -n 1 /highway_lane_strategy/state
rostopic echo -n 1 /perception/merge_gap/available
```

Run ROS commands inside the container after entrypoint setup.
The previous baseline is available with
`enable_highway_v9:=false` on the launch (for differential testing).
However this opt-out has not been separately validated under a real simulator.

### Known limitations and safety

- **NOT validated in MORAI**, only source/launch wiring and static checks.
- Roundabout V6.1 and V9 hand-off still require temporal field tests; do not
  test at full speed before proving basic arbitration and sensor freshness.
- V9 WAIT_GAP currently selects its adaptive path immediately; rate limits and
  lane/traffic safety are dependent on fresh V9 planner outputs.
- As in upstream, Competition status UDP 9081 and GPS/IMU interfaces must match
  the team's sender. Do not use the old 1911 Ego-status assumption.
- `run_test.sh` is container-oriented. The other existing scripts are unchanged.
