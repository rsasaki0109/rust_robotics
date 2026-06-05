# SafeDec-lite: STL-Shielded Navigation Decoding

Source theme: constrained decoding for safe robot navigation policies.

This is a pure-Rust, non-learned reproduction of the core SafeDec idea: a base
navigation *policy* proposes scores over discrete actions, and a *shield*
expressed in Signal Temporal Logic (STL) keeps the decoded action sequence
safe. No VLM or learned model is required — the shield is what makes the
behavior safe, and that part reproduces directly.

## Implemented slice

- A deterministic greedy goal-seeking base policy that scores each discrete
  action by how much it reduces Euclidean distance to the goal. On its own it
  happily drives straight through a hazard.
- A shield built from `rust_robotics_planning::stl_cbs` primitives:
  - an `always-avoid` geofence over a time interval, enforced as a **hard
    constraint** — candidates entering it (within an optional safety margin) are
    pruned from the beam;
  - an `eventually-reach` goal region, used as a **soft shaping reward** plus a
    feasibility filter on the final answer.
- `SafeDecoder::decode` runs a deterministic constrained beam search and returns
  both the greedy (unshielded) path and the shielded path, so the number of
  steps where the shield overrode the greedy choice — and the resulting
  robustness gain — is measurable.

## Primary files

- `crates/rust_robotics_planning/src/safe_decode_nav.rs`
- `crates/rust_robotics/examples/render_safe_decode_nav_svg.rs`
- `docs/assets/safe-decode-nav.svg`

## Run

```bash
cargo run -p rust_robotics --example render_safe_decode_nav_svg --no-default-features --features planning
```

## Result

On the bundled corridor with two geofences straddling the straight-line path,
the greedy policy cuts through the hazards (always-avoid robustness `-1.0`),
while the STL shield reroutes around both with a half-cell clearance
(always-avoid robustness `+1.0`), still reaching the goal region (eventually-
reach robustness `+1.0`) after 12 shield interventions versus the greedy
trajectory.

## Determinism

Ties are broken by action index and the beam is sorted with a stable total
order, so `decode` is fully deterministic. Tests cover the greedy-cuts-through
case, the shield-keeps-safe-and-reaches case, determinism, the no-hazard
no-intervention case, and input validation.

## Next useful extension

Replace the hand-written greedy policy with a sampled/learned action
distribution and add `eventually`/`until` task chains (visit A then B while
avoiding C), so the constrained decoder shields a richer temporal task.
