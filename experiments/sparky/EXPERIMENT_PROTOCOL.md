# Preregistered protocol — NOT EXECUTED

No measurements are contained in this file. These are acceptance criteria and experimental requirements for the next runnable environment, not claims of implemented instrumentation.

## Execution gates

1. Install Jazzy and Gazebo Harmonic. Preserve OS, CPU, package versions and package binary hashes. Pin Sparky to `ca39e50ba126a142f8ae7d806f8006ceea1f733a` and the candidate articulated Go2 simulator to `55ff151632a0220bf3ae0c0772392615d86da7a8`.
2. Build and run the simulator readiness scripts. The supplied readiness observer only checks message flow; passing it does not prove walking stability, sensor rendering, localization, collision sensing or navigation.
3. Confirm gait stability, then confirm a real sensor scan, monotonic `/clock`, distinct estimated odometry and ground truth, and exactly one publisher per dynamic TF edge. Ground truth must never feed localization or the controller.
4. Finish the Nav2 adapter. Reuse the exported resume configuration; connect controller output through a common limiter and collision monitor to CHAMP. Replace hardware transport only. Load AMCL and map_server, check all lifecycle nodes are active, and prove a real NavigateToPose action succeeds before comparing plugins.
5. Dump runtime parameters and plugin-load logs. YAML plugin names alone do not prove which controller executed. Fail the run if the requested and loaded plugins differ.
6. Only after the baseline succeeds, implement and validate fault injection and bounded prediction, then execute the ablation.

## Controlled navigation comparisons

Use NavFn (`nav2_navfn_planner::NavfnPlanner`, `use_astar=false`) and **SmacPlanner2D** (`nav2_smac_planner::SmacPlanner2D`). This choice resolves the ambiguous word SMAC; Hybrid and State Lattice are different experiments and must not be pooled with it.

Cross these with:

| Controller | Plugin |
|---|---|
| DWB | `dwb_core::DWBLocalPlanner` |
| RPP | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| MPPI | `nav2_mppi_controller::MPPIController`, DiffDrive motion model |

Use one world and its geometrically matching occupancy map, one articulated Go2 model, identical initial poses, goal poses and yaw, obstacle positions, costmap resolution (0.05 m), footprint and inflation layers in all arms. No map or world has yet been validated for these scenarios. Do not treat the repository's saved PGM as a matching Gazebo world without reconstructing and checking it.

Preserve the Sparky footprint `[(0.360,0.200),(0.360,-0.200),(-0.450,-0.200),(-0.450,0.200)]`. Record that this navigation footprint differs from the articulated simulator's physical collision meshes. Use the same physical meshes in every run.

Initial shared limits: forward 0.30 m/s, lateral 0, yaw rate 0.60 rad/s; acceleration 0.45 m/s² and 1.0 rad/s², deceleration 0.60 m/s² and 1.20 rad/s². Preserve DWB's native baseline first. A common command limiter is needed for a fair applied-command cap, especially for Jazzy RPP, whose parameter interface is not the same as current Rolling. Record raw controller and post-limiter commands separately and report clipping frequency. Configure the CHAMP actuator limits consistently.

Scenarios: straight, a forced 90-degree turn, narrow corridor, and obstacle avoidance. A diagonal free-space goal is not proof of a 90-degree turn. Distinguish a mapped static obstacle from an initially unmodeled obstacle. The source defaults disable live obstacles in the global costmap; turning them on is a separate declared configuration change.

Use at least three fresh simulator resets per condition; record random seeds and whether the installed MPPI build exposes deterministic seed control. If it does not, report that limitation. A full six-pair, four-scenario, three-repeat comparison is 72 attempted navigation runs, excluding setup failures. No runs have been attempted yet.

## Delay experiments

Fix NavFn+DWB and all navigation settings. Test 0, 100, 200, 300, 500 and 1000 ms in separate arms:

* **Odometry transport delay:** queue `/odom` after acquisition stamping; keep the dynamic TF edge undelayed. Do not delay lidar, ground truth or `/clock`.
* **TF publication delay:** queue only the selected `odom -> base_footprint` edge while preserving source stamps; keep `/odom` undelayed. The simulator also has `base_footprint -> base_link`; document the exact edge rather than claiming the whole tree was delayed. Remove the original publisher of the injected edge to prevent an undelayed duplicate.
* **Lookup-failure injection:** deliberately withhold the selected edge over documented windows, including a window longer than the TF cache, and record actual consumer failures. An old transform can remain available for `Time(0)`; dropping messages or making a separate probe fail does not prove Nav2 itself encountered lookup failures. Define duration and duty cycle explicitly; a 0-ms arm is the no-fault control.

Record intended delay and actual source-to-delivery age under simulated time; measure callback and processing durations under monotonic wall time. Never divide simulated-time delay by wall-clock runtime.

Keep acquisition time, receipt time and prediction target time as separate fields. Test source-stamp preservation separately from bounded prediction. A proposed predictor must reject nonfinite, future, duplicate and out-of-order data, reset on clock discontinuities, propagate covariance, and stop prediction beyond its declared horizon (initial candidate 0.20 s). It must not clamp a 1-second-old state to 0.20 s of propagation and then label it current. Beyond the horizon, expose stale status and use a declared safe-stop/failure policy. Validate the predictor against withheld ground-truth poses before calling it a validated correction.

The supplied bundle does **not** implement the delay node or predictor. Their behavior must not be inferred from the source audit or configuration export.

## Required evidence and metrics

Record `/clock`, ground-truth odometry, unmodified and delivered estimated odometry, `/tf`, `/tf_static`, scan, global paths, raw controller Twist, limiter output, collision-monitor output, joint controller commands/states, goal acceptance/result/status, lifecycle transitions, fault schedule, delivery receipts and processing timers. Save static TF and maps with transient-local-compatible recording QoS. Prefer MCAP when installed; verify replay before interpreting metrics.

Persist a run manifest with source commits, adapter hash, package versions, exact loaded plugins, runtime parameter dumps, world/map/model hashes, start/goal, seed, repetition, fault mechanism, intended delay and actual observed age. Missing metadata is a failed evidence gate.

* **Trajectory:** plot actual ground-truth and estimated poses, not integrated requested velocities. Transform them to the same verified frame. Do not align each run with a fitted transform that conceals localization error.
* **Cross-track error:** distance from each ground-truth sample to segments of the global path available at that sample's time. Preserve every replanned path and its timestamp; do not compare all samples to the final path.
* **Angular velocity:** show raw commands, applied commands and ground-truth yaw rate against simulated time.
* **Oscillation:** count signed angular-command reversals outside a declared deadband with a declared persistence interval. Exclude deliberate heading changes and report the definition plus raw traces.
* **Success:** actual action result SUCCEEDED, verified final ground-truth position/yaw, no collision. Report action success and physical arrival separately. Show numerator/denominator and confidence intervals. Zero attempted runs means N/A, never 0% success.
* **Collisions:** count contact episodes between robot and obstacles/walls, excluding normal foot-ground contact and declared self-contact. The candidate simulator exposes foot contact sensors; that alone is not whole-body collision coverage. Add and validate obstacle/body contact instrumentation first. No messages means unknown unless sensor health is proven.
* **Latency:** instrument planning and controller compute start/end with monotonic time or tracing. Inter-command period is not computation latency. Report p50/p95/p99 and missed control deadlines separately from intentional feedback delay.
* **Screenshots:** capture an actual Gazebo render or real RViz view from the run. Never use upstream README screenshots as this experiment's evidence.

Keep failed/aborted runs, startup logs, ROS bags, configuration files and raw sample tables. Plotting must refuse missing or empty evidence. Do not create placeholder bags, fabricated metrics, or illustrative trajectory images.
