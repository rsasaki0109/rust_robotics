# Distributed Formation Consensus via ADMM

A pure-Rust reproduction slice for the coordination layer of distributed MPC, in
the spirit of ACLM-style consensus control. It is the first distributed/
optimization-control target in this repo.

A team of agents must agree on a common formation center `z` while each agent
settles at `z + offset_i`, trades off staying near its own preferred position
`a_i`, and respects a per-agent box constraint. The agreement is reached by the
classic *consensus ADMM* iteration: each agent solves a tiny local proximal
problem, the team averages to a consensus center, and scaled dual variables drive
the per-agent residuals to zero.

## Problem

Minimize `sum_i (w_i/2) ||x_i - a_i||^2` subject to `x_i - offset_i = z` for all
agents and `lower_i <= x_i <= upper_i`.

With equal weights and no box constraints the consensus center has the closed
form `z* = mean(a_i - offset_i)`, which the solver recovers exactly. Box
constraints break the closed form and make the problem genuinely iterative — that
is where ADMM earns its keep.

## Algorithm

Consensus ADMM (scaled dual form), iterated to convergence:

1. **x-update (local).** Each agent independently solves its proximal step and
   projects onto its box:
   `x_i = clamp( (w_i a_i + rho (z + offset_i - u_i)) / (w_i + rho) )`.
2. **z-update (consensus).** The team averages: `z = mean_i (x_i - offset_i + u_i)`.
3. **dual update.** `u_i += x_i - z - offset_i`.

The primal residual is `sqrt(sum_i ||x_i - z - offset_i||^2)` and the dual
residual is `rho * sqrt(N) * ||z - z_prev||`; the iteration stops once both fall
below the tolerance.

- [`AgentSpec`] — one agent's preferred position, formation offset, weight, and
  box bounds (`with_box`, `with_weight`).
- [`AdmmConfig`] — penalty `rho`, iteration cap, residual tolerance.
- [`solve_formation_consensus`] — runs the iteration and returns the agent
  positions, the consensus center, and the primal/dual residual history.

## Benchmark

`benchmark_admm_formation` places six agents that want a regular hexagon around a
shared center, each pulled toward a scattered preferred position. The two
right-side agents are confined to a left corridor (a box constraint), so their
formation slots cannot sit where the unconstrained average would put them and the
whole consensus center is pulled left to keep them inside it. The benchmark writes
`docs/assets/admm-formation.csv` and `.svg` (preferred positions, the final
formation polygon and center, the corridor, and a log-scale residual inset):

```bash
cargo run -p rust_robotics --example benchmark_admm_formation --no-default-features --features control
```

On the bundled instance the consensus center settles at about `(-1.20, -0.03)` —
shifted left by the corridor — and both residuals fall below `1e-7` in about 91
iterations (versus ~25 when the corridor does not bind). The next extension is a
receding-horizon variant (consensus over short trajectories rather than static
positions) and edge-wise consensus on a communication graph rather than a global
average.
