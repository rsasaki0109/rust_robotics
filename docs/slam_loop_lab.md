# SLAM loop closure lab

[Open the lab](https://rsasaki0109.github.io/rust_robotics/playground/?tab=slam&algorithm=loop&loop_view=compare&loop_event=first&playing=0&loop_vectors=1).

![The same LiDAR scans at front-end and pose-graph poses](assets/playground/slam-loop.png)

Two maps use **the same recorded scans at the same update**. The orange map
places them at independent scan-to-map front-end poses. The cyan map places
them at the pose-graph poses recorded at that update. Both use the same world
scale. The white ring marks the simulated true robot; the red points are the
current scan.

## Try it

- Choose **Before first**, then **+1 update**. One accepted loop match can
  re-optimize many older nodes. Enable **Node correction arrows** to see their
  cumulative shifts from the front-end poses.
- Use **Next loop** and **Previous loop** to visit actual accepted matches.
  **Back** moves one update backward, restoring the map that existed then.
- Tap or drag the error graph, or use the update slider. Both maps, position
  errors, edges, and diagnostics use that one selected update. **Play** runs
  the recording; **Result** selects its last recorded update.
- Choose **Repeated: check off**, then **First false loop**. Repeated pillars
  can produce a plausible scan match to the wrong place. A low registration
  residual alone does not establish that the place match is correct.
- Choose **Repeated: check on**. These two scenarios use identical truth,
  odometry, scans, and independent front-end estimates; only the ambiguity
  check changes. Compare their accepted edges and graph errors.

The corridor recording has 489 updates. At its first accepted loop, update
412, the current robot's position error drops from about 0.42 m in the front
end to 0.01 m in the graph estimate. In the repeated-pillar recording, 12 of
16 accepted edges are false with the check off; all 7 accepted edges are true
with it on. These are outcomes of these deterministic simulated scenarios,
not general performance guarantees.

## What is recorded

The graphs contain both complete position-error histories, measured against
simulation truth. Thin purple or gold lines mark accepted loop events. Purple
edges are true matches; gold edges are false matches according to the simulated
relative pose (translation error greater than 0.5 m).

Both maps use only scans belonging to nodes that already exist at the selected
update. The right map uses that update's stored node poses. Later graph
corrections and the separate final optimization are not applied to earlier
frames. Seeking backward also removes later edges and nodes.

Gold arrows show **cumulative node displacement** from the front-end map to
the current graph map. They do not isolate the displacement of just the latest
optimization. A newly accepted edge need not trigger re-optimization if its
innovation is small; the event diagnostics say whether optimization happened.

False-match labels use simulation ground truth, which is unavailable to a real
robot. **First false loop** is a teaching aid for this recorded experiment.
Playback advances at 40 recorded updates per second; this is a playback rate,
not a physical clock for the robot. Pauses freeze the map and background gaps
of at least one second are discarded.

**Share** reconstructs the same scenario, update, and arrow setting, paused.
**Single map** returns to the original replay, and **Compare maps** returns to
the comparison at the same update. Drive (live), EKF-SLAM, FastSLAM, and ICP
remain available in the same SLAM tab.

## Reproduce

```sh
cargo test --locked -p rust_robotics_playground slam:: --bin rust_robotics_playground
```

The lab reuses the library's [deterministic corridor recordings](../crates/rust_robotics_slam/src/lidar_loop_scenario.rs)
and [LiDAR graph SLAM](../crates/rust_robotics_slam/src/lidar_graph_slam.rs).
The [browser checks](../scripts/web_smoke/slam_loop.mjs) use real mouse and
touch input and independently verify errors, map transforms, node shifts,
prefix-only edges, ground-truth match classification, and exact shared state.
