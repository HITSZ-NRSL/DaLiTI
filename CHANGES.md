# Changes

## Robustness
- `laserMapping` no longer crashes on empty LiDAR scans (LiDAR fully blocked, e.g. by dense smoke); such scans are
  processed as degraded frames.
- `imuPreintegration` no longer aborts under CPU load (it read an empty odometry queue); invalid odometry input is
  ignored and failed graph resets re-initialize locally.
- IMU processing no longer uses uninitialized values at start-up (this could build an empty first map).
- `run_demo.sh` no longer ends a run early when a single playback check fails.

## Fusion while the LiDAR is degraded
- The IMU always propagates the state. Gyro / accelerometer biases and gravity are filter states; the world frame is
  gravity-aligned at start-up, and the start-up mean gyro is used as bias only if the platform is still.
- While the LiDAR is degraded (update stopped, or normalized LiDAR factor below `vio_fusion_max_zeta_l`), the motion
  of the thermal-inertial odometry between two LiDAR frames (time-aligned, in its body frame, gated) is fused as a
  position / velocity measurement. It never corrects the attitude; the gyro keeps the heading.
- LiDAR point noise is scaled by `1 / zeta_l_norm`.
- While degraded: ground-robot non-holonomic constraint and a gravity-referenced roll / pitch correction.

## Running and output
- `run_demo.sh`: `--profile` (`indoor-smoke`, `outdoor-smoke`, `rooftop`), `--start` / `--duration`, `--camera`,
  `--gt` (ATE after the run), automatic image build per git commit (retried after network errors), `--image`,
  `--rebuild`.
- All profiles start at 0 seconds and play to the end by default. `outdoor-smoke` uses the prepared
  `daliti-outdoor-smoke.bag`; `rooftop` uses the original untrimmed bag and its raw thermal image topic.
- Input image topics are configured in `daliti/config/<profile>_input.yaml` (`img_topic`).
- TUM poses are stamped with the time of the state (end of each LiDAR scan), as on `/aft_mapped_to_init`.
- Rooftop camera calibration: `daliti/config/daliti-rooftop_camera.yaml`.

## Results
Docker, `run_demo.sh`, desktop PC (Ryzen 9 9950X); previous version = release 97a1cb4.

| sequence | metric | previous version | this version |
|---|---|---|---|
| indoor smoke, whole bag | start-to-end loop error* | 7.6 - 10.9 m (3 runs) | **0.03 - 0.04 m** (4 runs) |
| rooftop smoke, 90 s - 177 s (back near the start point), `--profile rooftop --start 90 --duration 87` | ATE RMSE (SE(3) aligned) | crashes at 116 s (with only the crash fixed: 2.8, 124, 199 m) | **0.40 - 1.29 m** (6 runs, median 0.45 m) |

\* The robot stops 0.34 m from its start; the error compares the estimated start-to-end transform with the one
obtained by registering the first and the last (static) LiDAR scans.

On a heavily loaded machine the previous version's `imuPreintegration` aborted 6 - 14 times per indoor run; with the
fix included here there were none in 9 runs.

## Main parameters (`eskf_lio/config/feat.yaml`, `mapping:`)
| parameter | default | |
|---|---|---|
| `vio_fusion_enable` | true | fuse the thermal-inertial motion while the LiDAR is degraded |
| `lidar_weight_by_zeta` | true | LiDAR point noise / normalized LiDAR factor |
| `nhc_enable` | true | ground-robot constraint while degraded (platform drives along the IMU x axis) |
| `tilt_correction_enable` | true | roll / pitch correction while the LiDAR update is stopped |
| `gravity_align` | true | world z axis up at start-up |
| `fusion_debug_log` | false | one diagnostic log line per LiDAR frame |

## Known limitations
- The thermal-inertial odometry needs the camera calibration of the session (`--camera`).
- `nhc_enable` assumes the platform drives along the IMU x axis; disable it for other platforms or mountings.
- If the platform moves during start-up, the gyro bias is assumed to be zero.
- Rooftop: most of the remaining error builds up at the end of the LiDAR blackout (about 130 - 140 s), where the
  thermal-inertial motion overestimates the speed; how much varies between runs.
